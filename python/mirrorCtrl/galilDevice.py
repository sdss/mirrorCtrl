from __future__ import division, absolute_import
"""Galil Device.

This talks to a Galil motor controller, which commands actuators to move the mirror and
reads from encoders to determine mirror position.

There is lots of info at:
http://www.apo.nmsu.edu/Telescopes/HardwareControllers/GalilMirrorControllers.html#InterfaceReplies

To Do:
- ConstRMS keyword - how close the mirror can get to commanded orientation?
- switch to pyparsing for galil replies?

Notes:
Update Galil Code Constants:
2.5m secondary, turn off piezo corrections, we handle them here now
All mirrors w/ encoders, disable subsequent moves, we handle them here now
"""
import time
import itertools
import re

import numpy
from RO.StringUtil import quoteStr, strFromException
from RO.SeqUtil import asSequence
from RO.Comm.TwistedTimer import Timer
from twistedActor import TCPDevice, UserCmd, log

from .const import convOrient2UMArcsec

__all__ = ["GalilDevice", "GalilDevice25Sec"]

## a blank userCmd that is already set as done
def getNullUserCmd(state):
    nullUserCmd = UserCmd(userID=0, cmdStr="Null UserCmd")
    nullUserCmd.setState(state)
    return nullUserCmd

## Stop command for direct galil communication
GalCancelCmd = re.compile('^ST$', re.IGNORECASE)
## Reset command for direct galil communication
GalResetCmd = re.compile('^RS$', re.IGNORECASE)
## RegEx stuff up here to keep these from being needlessly re-compiled...
StartsWithNumRegEx = re.compile(r'^-?[0-9]')
## find numbers not preceeded by a '/' or a letter and not followed by a letter
GetDataRegEx = re.compile(r'(?<!/|[A-Za-z])[0-9-.]+(?![A-Za-z])')
## params are uppercase
ParamRegEx = re.compile(r'^-?[A-Z]')
## Time estimate regex
TimeEstRegEx = re.compile(r'^sec +to|^max +sec|^time +for')
## Match an 'OK' line
OKLineRegEx = re.compile(r'^OK$', re.IGNORECASE)
## Version regex
DevSpecVersionRegEx = re.compile(r"version of .+ additions", re.IGNORECASE)
## Matches the returning command echo
CmdEchoRegEx = re.compile(r'xq *#[a-z]+$', re.IGNORECASE)
## Matches the echo returning from commanded axes
AxisEchoRegEx = re.compile(r'[A-Z]=-?(\d+)', re.IGNORECASE)



################## KEYWORD CAST DEFINITIONS ##################################
def floatCast(number):
    """Cast a float into a string

        @param[in] float: a float
        @return a string
    """
    if numpy.isnan(number):
        return "NaN"
    else:
        return "%.2f" % number

def mountCast(mount):
    """ Cast a mount array into a string.
        @param[in] mount: Array of mount values
        @return a string
    """
    return ",".join([floatCast(x) for x in mount])

def orientCast(orient):
    """ Cast an orientation array into a string.
        @param[in] orientation: numpy array of orientation values in mm and radians
        @return a string of orientation (in user friendly units, um and arcsec)
    """
    return ",".join([floatCast(x) for x in convOrient2UMArcsec(orient)])

def strArrayCast(strArray):
    """Turn an array of strings into a single string, comma separated.
    @param[in] strArray: an array of strings
    @return a string
    """
    return ",".join([str(x) for x in strArray])

def intOrNan(anInt):
    """Return an string int or "nan"
    @param[in] anInt: either an integer or a nan
    @return a string
    """
    try:
        return str(int(anInt))
    except ValueError:
        return "nan"

def statusCast(status):
    """Return a
    @param[in] status: an array of status bits
    @ return a string, status concatenated into string of comma separated values which are possibly nans
    """
    statusStr = []
    for element in status:
        statusStr.append(intOrNan(element))
    return ",".join(statusStr)
###########################################################################

class GalilTimer(object):
    """Measure elapsed time
    """
    def __init__(self):
        """Construct a GalilTimer in the reset state
        """
        self.reset()

    def reset(self):
        """Reset (halt) timer; getTime will return nan until startTimer called
        """
        self.initTime = numpy.nan

    def startTimer(self):
        """Start the timer
        """
        self.initTime = time.time()

    def getTime(self):
        """Return elapsed time since last call to startTimer

        Return nan if startTimer not called since construction or last reset.
        """
        return "%.2f"%(self._getTime())

    def _getTime(self):
        return time.time() - self.initTime

class GalilStatus(object):
    """A container for holding the status of the Galil """
    # State options
    movingState = "Moving"
    doneState = "Done"
    homingState = "Homing"
    failedState = "Failed"
    notHomedState = "NotHomed"
    def __init__(self, device):
        """Construct a GalilStatus

        @param[in] device: mirror device
        """
        self.mirror = device.mirror
        ## number of actuators
        self.nAct = len(self.mirror.actuatorList)

        # all the Status keyword/value pairs we will cache, and their initial values
        self.maxDuration = 0
        self.duration = GalilTimer()
        # actuator mount determined from a measured orientation
        self.actMount = numpy.asarray([numpy.nan]*self.nAct) # get rid of?
        # desired encoder mount determined from desired orientation
        self.desEncMount = numpy.asarray([numpy.nan]*self.nAct)
        # encoder mount reported from the galil
        self.encMount = numpy.asarray([numpy.nan]*self.nAct)
        # the user commanded mount position
        self.modelMount = numpy.asarray([numpy.nan]*self.nAct)
        # subsequent commanded mount positions determined during iterations
        self.cmdMount = numpy.asarray([numpy.nan]*self.nAct)
        # offset applied to a previously commanded mount, a new mountErr is determined each iteration
        self.mountErr = numpy.asarray([0.]*self.nAct)
        # last computed total offset between the first commanded mount position and the last commanded mount position after iteration
        self.netMountOffset = numpy.asarray([0.]*self.nAct)
        # measured orientation (based on encoder lengths)
        self.orient = numpy.asarray([numpy.nan]*6) # get rid of?
        # orientation based on commanded actuator positions
        self.mountOrient = numpy.asarray([numpy.nan]*6)
        # desired orientation, user specified, but adjusted for induced motions due to fixed links.
        self.desOrient = numpy.asarray([numpy.nan]*6)
        # age of desired orientation
        self.desOrientAge = GalilTimer()
        # current move iteration
        self.iter = 0
        # max number allowed for move iterations
        self.maxIter = device.MaxIter if self.mirror.hasEncoders else 1
        # status bits
        self.status = numpy.asarray([numpy.nan]*self.nAct)
        # acutators which are currently moving
        self.moving = 0.
        # acutators which are currently homing
        self.homing = numpy.asarray(["?"]*self.nAct)
        # actuators which are currently homed
        self.axisHomed = numpy.asarray(["?"]*self.nAct)
        # dictionary containing casting strageties to output current gailil status/state info
        self.castDict = {
            "nAct": int,
            "maxDuration": floatCast,
            "duration": self.duration.getTime,
            "actMount": mountCast,
            "desEncMount": mountCast,
            "encMount": mountCast,
            "modelMount": mountCast,
            "cmdMount": mountCast,
            "mountErr": mountCast,
            "netMountOffset": mountCast,
            "orient": orientCast,
            "desOrient": orientCast,
            "mountOrient": orientCast,
            "desOrientAge": self.desOrientAge.getTime,
            "iter": intOrNan,
            "maxIter": intOrNan,
            "status": statusCast,
            # "moving": str,
            "homing": strArrayCast,
            "axisHomed": strArrayCast,
        }

    def _getKeyValStr(self, keywords):
        """Package and return current keyword value info in status cache

        @param[in] keywords: a list of keywords for which you wish to return a formatted string.

        @return statusStr: string in the correct keword value format to be sent to users
        """
        strList = []
        for keyword in keywords:
            if keyword in ['duration', 'desOrientAge']:
                strVal = self.castDict[keyword]()
            else:
                val = getattr(self, keyword)
                strVal = self.castDict[keyword](val)
            strList.append("%s=%s" % (keyword, strVal))
        return "; ".join(strList)

    def _getCurrentState(self, cmd=None):
        """Return the state of the device
        @param[in] cmd, a command whos state to watch for failure.
        """
        # are any axes homing?
        if 1 in self.homing:
            return self.homingState
        # is it moving?
        elif self.moving:
            return self.movingState
        elif 0 in self.axisHomed:
            return self.notHomedState
        elif cmd and cmd.didFail:
            return self.failedState
        return self.doneState

    def currentStatus(self, cmd=None):
        """Construct a keyword summarizing the state of the device
        @param[in] cmd, a command whos state to watch for failure.
        """
        keyword = "state"
        state = self._getCurrentState(cmd)
        nIter = "%i" % self.iter
        maxIter = "%i" % self.maxIter
        totDuration = floatCast(self.maxDuration)# if state in [self.movingState, self.homingState] else 0)
        remDuration = floatCast(0)
        if self.maxDuration:
            duration = self.duration._getTime()
            if not numpy.isnan(duration):
                remDuration = floatCast(self.maxDuration - duration)

        # remDuration = floatCast(self.maxDuration - self.duration._getTime() if state in [self.movingState, self.homingState] else 0)
        # print keyword + "=" + ",".join([state,nIter,maxIter,remDuration,totDuration])
        return keyword + "=" + ",".join([state,nIter,maxIter,remDuration,totDuration])


class GalilDevice(TCPDevice):
    """The Galil Device Object

    To do: enforce devCmd time limits.
    Do that locally or have DeviceCommand do it???
    The trick for the latter is knowing when to start the timer,
    but doing so when the command begins is probably sufficient,
    or when the state is Running.
    """
    ## initial timeout to use for every device command
    ## the following are thresholds (large moves) beyond which automatic (previously computed)
    ## move offsets should not be used
    DevCmdTimeout = 2.
    ## scale the determined move offset correction by this much, eg go only 90%
    CorrectionStrength = 0.9
    def __init__(self, mirror, host, port, maxIter=5, callFunc=None, name=None):
        """Construct a GalilDevice

        @param[in] mirror: an instance of mirrorCtrl.MirrorBase
        @param[in] host: host address of Galil controller
        @param[in] port: port of Galil controller
        @param[in] maxIter: the maximum number of iterations to refine the mirror position
        @param[in] callFunc: function to call when state of device changes;
            it receives one argument: this device.
            Note that callFunc is NOT called when the connection state changes;
            register a callback with "conn" for that.
        @param[in] name: name of device; if None then use mirror.name
        """
        self.MaxIter = maxIter
        self.mirror = mirror
        TCPDevice.__init__(self,
            name = name if name else mirror.name,
            host = host,
            port = port,
            callFunc = callFunc,
            cmdInfo = (),
        )
        ## currently executing (or last completed) device command
        self.currDevCmd = self.cmdClass("")
        self.currDevCmd.setState(self.currDevCmd.Done)
        ## currently executing (or last completed) user command
        self.userCmd = getNullUserCmd(state=UserCmd.Done)
        ## holds information sent from the galil in response to the current device command
        self.parsedKeyList = []
        ## number of actuators
        self.nAct = len(self.mirror.actuatorList)
        ## valid axes for this mirror/galil
        self.validAxisList =  ('A', 'B', 'C', 'D', 'E', 'F')[0:self.nAct]
        ## dictionary of axis name: index, e.g. A: 0, B: 1..., F: 5
        self.axisIndexDict = dict((axisName, ind) for ind, axisName in enumerate(self.validAxisList))
        ## a galil status object
        self.status = GalilStatus(self)
        self._inDevCmdCallback = False

    # def logMsg(self, msg):
    #     """Temporary log routine, incase of logging prior to actor
    #     """
    #     print 'temp log', msg

    @property
    def userCmdOrNone(self):
        """return self.userCmd if not done, else None
        """
        if self.userCmd.isDone:
            return None
        return self.userCmd

    def formatAsKeyValStr(self, keyword, dataList):
        """Format a keyword=value pair string.

        @param[in] keyword: a keyword (string)
        @param[in] dataList: the associated list of values

        @return outStr: a string in the correct format to be sent to the user
        """
        valStr = ', '.join(str(v) for v in asSequence(dataList))
        outStr = keyword + '=' + valStr
        return outStr

    def init(self, userCmd=None, timeLim=None, getStatus=False):
        """Initialize Galil

        @param[in] userCmd: user command that tracks this command, if any
        @param[in] timeLim: IGNORED maximum time before command expires, in sec; None for no limit
        @param[in] getStatus: IGNORED (status is not output)
        @return devCmd: the device command that was started (and may already have failed)

        Called on disconnection
        """
        log.info("%s.init(userCmd=%s, timeLim=%s, getStatus=%s)" % (self, userCmd, timeLim, getStatus))
        self.cmdStop(userCmd=userCmd, getStatus=getStatus)

    def parseReply(self, replyStr):
        """Parse a reply from the Galil and seperate into key=value format.

        Any descriptive text following the data is a key. There are two possibilities:
        - One key for all values: true if the key contains no commas
        - A separate key for each value: true if the key contains a comma

        @param[in] replyStr: Galil reply string

        @return two values:
        - keyList: list of keys: one per element of dataListList
        - dataList: list of data strings:
            - if keyList contains one element, then all data is for that key
            - otherwise there will be one item of data per key

        example lines:
            0000.2,  0362.7,  0000.2,  0000.0,  0000.0 max sec to find reverse limit switch
             0300.1,  0300.1,  0300.1,  0000.0,  0000.0 max sec to find home switch
             0008.2,  0008.2,  0008.2,  0000.0,  0000.0 sec to move away from home switch
            Finding next full step
             041,  006.6 microsteps, sec to find full step
            -000006732,  000014944,  000003741,  999999999,  999999999 position error
             1,  1,  1,  0,  0 axis homed
        """
        # print "parseReply(replyStr=%r)" % (replyStr,)
        # Grab the data
        # Match only numbers (including decimal pt and negative sign) that are not preceeded by '/'
        # or immediately surrounded by any letters letters.
        # Eg: 'RNGx/2' which I don't want to match
        dataList = GetDataRegEx.findall(replyStr) # will put non-overlapping data (numbers) in a list
        numVals = len(dataList)
        # Grab descriptor info which follows data.
        # split on whitespace n times, last split will be descriptor info
        textOnly = re.split(r' +', replyStr, maxsplit = numVals)[-1]
        # if there are multiple descriptors, they will be separated by a ','
        # or whitespace following a '?'
        # ....so split the text further
        keyList = re.split(r',|(?<=\?) ', textOnly)
        # remove leading and trailing whitespace from each key
        keyList = [key.strip() for key in keyList]

        # Make sure we have either 1 key with multiple values (usually one per actuator)
        # or N keys and the same number of values
        if len(keyList) > 1 and len(keyList) != len(dataList):
            self.writeToUsers("w", "UnparsedReply=%s; Text=\"number keys does not match number of values\"" %\
                (quoteStr(replyStr),), cmd=self.userCmdOrNone)
            return None
        return keyList, dataList

    def sendGalilParam(self, key, dataList):
        """Write a Galil parameter (without recording anything)

        @param[in] key: Galil description key for data; a string that probably includes spaces;
            the first word is used to generate the output keyword
        @param[in] dataList: list of values

        In most cases the output keyword will be Galil<firstWord>
        where firstWord is the first word in key with any trailing "x" removed.
        however -RNGx/2 is ignored and RNGx/2 is turned into HalfRNG.
        """
        param = key.split()[0] # in example: just keep -RNGx/2
        if param == '-RNGx/2':
            # skip it
            return
        elif param == 'RNGx/2':
            param = 'HalfRNGx'
            dataList = [int(val) for val in dataList]
        if param.endswith("x"):
            param = param[:-1]
        keyword = 'Galil%s' % (param,)
        outStr = self.formatAsKeyValStr(keyword=keyword, dataList=dataList)
        self.writeToUsers("i", outStr, cmd=self.userCmdOrNone)

    def actOnKey(self, key, dataList, replyStr):
        """Process data associated with one key

        @param[in] key: parsed descriptive string returned by self.parseReply
        @param[in] dataList: list of data associated with key
        @param[in] replyStr: unparsed reply, for error messages
        """
         #line begins with cap letter (or -cap letter)
        if ParamRegEx.match(key):
            # this is a Galil parameter, format it and send it to the user
            self.sendGalilParam(key, dataList)

        elif 'software version' in key:
            # needs a special name so don't use sendGalilParam
            msgStr = 'Galil%s=%s' % ('SoftwareVersion', dataList[0])
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)

        elif DevSpecVersionRegEx.match(key):
            # device-specific software version
            msgStr = '%s=%s' % (('deviceSoftwareVersion', dataList[0]))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)
            return

        elif TimeEstRegEx.match(key):# and ("full step" not in key): # ignore sec to find full step, cuts the timeout short!
            # contains information about estimated execution time
            # update status and notify users
            dataList = numpy.asarray(dataList, dtype=float)
            maxTime = numpy.max(dataList) # get time for longest move
            self.status.maxDuration = maxTime
            # reset the timer
            self.status.duration.startTimer()
            # updateStr = self.status._getKeyValStr(["maxDuration"])
            # # append text describing time for what
            # updateStr += '; Text=%s' % (quoteStr(key),)
            # self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)
            self.sendState(cmd=self.userCmdOrNone)
            # self.writeToUsers("i", self.status.currentStatus(), cmd=self.userCmdOrNone)
            # adjust time limits
            #if not self.currDevCmd.isDone:
            log.info("New timeout: %0.2f for device cmd %r"%(maxTime+4, self.currDevCmd))
            self.currDevCmd.setTimeLimit(maxTime + 4)
            #if not self.userCmd.isDone:
            log.info("New timeout: %0.2f for user cmd %r"%(maxTime+6, self.userCmd))
            self.userCmd.setTimeLimit(maxTime + 6)

        elif ('commanded position' == key) or ('target position' == key):
            # modelMount must not be updated for actuator error determination
            # and subsequent move commands
            pass

        elif ('actual position' == key) or ('final position' == key):
            # measured position (adjusted)
            # account for encoder --> actuator spatial difference
            self.status.encMount = [int(x) if int(x) != 999999999 else numpy.nan for x in dataList]
            updateStr = self.status._getKeyValStr(["encMount"])
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)
            # DesOrient may be nans
            if not numpy.isfinite(sum(self.status.encMount[0:self.nAct])):
                # encoder positons contain NaN(s)
                # Further computations cannot be done
                self.status.orient = [numpy.nan]*6
                self.status.actMount = [numpy.nan]*self.nAct
            else:
                # initial guess is desOrient, unless nothing is there, in which case start from zero
                initOrient = self.status.desOrient if numpy.isfinite(sum(self.status.desOrient)) else numpy.zeros(6)
                orient = self.mirror.orientFromEncoderMount(self.status.encMount[0:self.nAct], initOrient)
                actMount = self.mirror.actuatorMountFromOrient(orient)
                actMount = numpy.asarray(actMount, dtype=float)
                # add these new values to status
                self.status.orient = numpy.asarray(orient[:], dtype=float)
                self.status.actMount = actMount[:]
            updateStr = self.status._getKeyValStr(["orient", "actMount", "cmdMount"])
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)

        elif key == 'axis homed':
            dataList = [int(num) for num in dataList]
            self.status.axisHomed = dataList
            updateStr = self.status._getKeyValStr(["axisHomed"])
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)

        elif key == 'status word':
            dataList = [int(num) for num in dataList]
            self.status.status = dataList
            updateStr = self.status._getKeyValStr(["status"])
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)

        else:
            # unrecognized key
            dataStr = ", ".join(str(val) for val in dataList)
            msgStr = "unknownReplyKey=%s, %s, %s" % (quoteStr(key), quoteStr(dataStr), quoteStr(replyStr))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)

    def handleReply(self, replyStr):
        """Handle a line of output from the device. Called whenever the device outputs a new line of data.

        @param[in] replyStr:  the reply, minus any terminating \n

        Tasks include:
        - Parse the reply
        - Manage the pending commands
        - Output data to users
        - Parse status to update the model parameters
        - If a command has finished, call the appropriate command callback
        """
        # print "handleReply(replyStr=%r); currDevCmd=%r" % (replyStr, self.currDevCmd)
        log.info("%s read %r, currDevCmd: %r" % (self, replyStr, self.currDevCmd))
        replyStr = replyStr.replace(":", "").strip(' ;\r\n\x01\x03\x18\x00')
        if self.currDevCmd.isDone:
            log.info("Ignoring unsolicited output from Galil: %s " % replyStr)
            return
        if self.currDevCmd.state == self.currDevCmd.Cancelling:
            raise log.error("Should not be set to Cancelling: %s"%self.currDevCmd)
            return

        #replyStr = unicode(replyStr, errors='ignore')
        #replyStr = replyStr.encode("ascii", errors = "ignore")

        #replyStr = replyStr.strip(' ;\r\n')
        if replyStr == "":
            # ignore blank replies
            return
        catchGOPOS = replyStr.startswith("?GOPOS")
        if catchGOPOS:
            # catch on full step error. Report it but don't fail the command
            self.writeToUsers("w", "Text=\"On Full Step Error: %s\"" % (replyStr,), cmd = self.currDevCmd)
            return
        elif replyStr.startswith("?"):
            # set state to failing, wait for 'OK', then fully fail
            # if cmd is failed instantly, the device could attribute the
            # following 'OK' as a reply for a different command.
            self.currDevCmd.setState(self.currDevCmd.Failing, textMsg=replyStr)
            self.writeToUsers("w", "Text=\"Device Command %s Failing: %s\"" % (self.currDevCmd.cmdStr, replyStr,), cmd = self.currDevCmd)
            return
        if OKLineRegEx.match(replyStr):
            # command finished
            state = self.currDevCmd.state
            if state == self.currDevCmd.Failing:
                # got the OK, now fail the command
                self.currDevCmd.setState(self.currDevCmd.Failed)
            else:
                self.currDevCmd.setState(self.currDevCmd.Done)
            return
        if CmdEchoRegEx.search(replyStr) or AxisEchoRegEx.search(replyStr) or GalCancelCmd.search(replyStr) or GalResetCmd.search(replyStr):
            # this is just the command echo (including possibly ST or RS) ignore it
            return
        if not StartsWithNumRegEx.match(replyStr):
            # line doesn't begin with a data value, eg 'Finding next full step'
            # show it to the user and return
            self.writeToUsers("i", "UnparsedReply=%s" % (quoteStr(replyStr),), cmd = self.userCmdOrNone)
            return

        # if we made it this far, separate the data from the descriptive text
        parsedLine = self.parseReply(replyStr)
        if not parsedLine:
            self.writeToUsers("w", "UnparsedReply=%s" % (quoteStr(replyStr),), cmd=self.userCmdOrNone)
            return

        keyList = parsedLine[0]
        dataList = parsedLine[1]
        if len(keyList) == 1:
            key = keyList[0]
            self.actOnKey(key=key, dataList=dataList, replyStr=replyStr)
            self.parsedKeyList.append(key)
        else:
            for key, data in itertools.izip(keyList, dataList):
                self.actOnKey(key=key, dataList=[data], replyStr=replyStr)
                self.parsedKeyList.append(key)

    def runCommand(self, userCmd, galilCmdStr, nextDevCmdCall=None, forceKill=False):
        """Begin executing a device command (or series of device commands) in reponse to a userCmd.

        @param[in] userCmd: a user command, passed from the mirrorCtrl
        @param[in] nextDevCmdCall: None, or callable.  Callable to be executed upon the sucessfull completion of the device command
        @param[in] forceKill: bool. Should this command kill any currently executing command

        @raise RuntimeError if a userCommand is currently executing and a forceKill is is not requested.
        """
        # print "%s.runCommand(userCmd=%r, galilCmdStr=%r, nextDevCmdCall=%r, forceKill=%r)" % (self, userCmd, galilCmdStr, nextDevCmdCall, forceKill)
        # print "    self.currDevCmd=%r" % (self.conn.state,)
        # print "    self.userCmd=%r" % (self.userCmd,)
        # print "    self.conn.state=%r" % (self.conn.state,)
        if not self.userCmd.isDone:
            if forceKill:
                # self.userCmd.setState(self.userCmd.Failed, textMsg="%r command forceKilled by incomming %r command"%(self.userCmd, userCmd))
                # print 'SAW FORCEKILL: curr %r, next: %r, devCmd: %r'%(self.userCmd, userCmd, self.currDevCmd)
                self.clearAll() # need dev command to finish before rest of code here is executed
                # self.userCmd.setState(self.userCmd.Cancelled, textMsg="%s cancelled by RS or ST"%self.userCmd.cmdStr)
            else:
                raise RuntimeError("User command collision! %s blocked by currently running %s!"%(userCmd.cmdStr, self.userCmd.cmdStr))
        if userCmd is None:
            userCmd = getNullUserCmd(UserCmd.Running)
        elif userCmd.state == userCmd.Ready:
            userCmd.setState(userCmd.Running)
        self.userCmd = userCmd
        self.userCmd.addCallback(self._userCmdCallback)
        self.startDevCmd(galilCmdStr, nextDevCmdCall)

    def startDevCmd(self, galilCmdStr, nextDevCmdCall=None):
        """
        @param[in] galilCmdStr: string, to be sent directly to the galil.
        @param[in] nextDevCmdCall: Callable to execute when the device command is done.
        """
        # print "%s.startDevCmd(galilCmdStr=%r, nextDevCmdCall=%r)" % (self, galilCmdStr, nextDevCmdCall)
        log.info("%s.startDevCmd(%r, nextDevCmdCall=%s)" % (self, galilCmdStr, nextDevCmdCall))
        if not self.currDevCmd.isDone:
            raise RuntimeError("Device command collision: userCmd=%r, currDevCmd=%r, desired galilCmdStr=%r" % \
                (self.userCmd, self.currDevCmd, galilCmdStr))
        self.currDevCmd = self.cmdClass(galilCmdStr, timeLim = self.DevCmdTimeout, callFunc=self._devCmdCallback, dev=self)
        self.nextDevCmdCall = nextDevCmdCall
        self.parsedKeyList = []
        # not self.clearAll()?
        try:
            if self.conn.isConnected:
                log.info("%s writing %r" % (self, galilCmdStr))
                self.conn.writeLine(galilCmdStr)
                self.currDevCmd.setState(self.currDevCmd.Running)
            else:
                self.currDevCmd.setState(self.currDevCmd.Failed, "Not connected")
        except Exception, e:
            self.currDevCmd.setState(self.currDevCmd.Failed, textMsg=strFromException(e))

    def replaceDevCmd(self, galilCmdStr, nextDevCmdCall=None):
        """Replace the current device command, set the previous one to done. And remove it's callbacks, so it's finished state
        will not effect the currend user command.

        @param[in] galilCmdStr: string, to be sent directly to the galil.
        @param[in] nextDevCmdCall: Callable to execute when the device command is done.
        """
        if not self.currDevCmd.isDone:
            self.currDevCmd._removeAllCallbacks()
            self.currDevCmd.setState(self.currDevCmd.Done)
        self.startDevCmd(galilCmdStr, nextDevCmdCall)

    def sendState(self, cmd=None):
        """Send the state of the galil to users.
        @param[in] cmd, a command this state is associated with, useful to register this method as a callback.
            upon state change.
        """
        self.writeToUsers("i", self.status.currentStatus(cmd), cmd=cmd)

    def _connCallback(self, conn=None):
        """If the connection closes, fail any current command
        """
        TCPDevice._connCallback(self, conn)
        if not self._ignoreConnCallback:
            if not self.conn.isConnected and not self.currDevCmd.isDone:
                self.currDevCmd.setState(self.currDevCmd.Failed, "Connection closed")

    def _userCmdCallback(self, userCmd):
        """Callback to be added to every user command

        @param[in] userCmd: a user command, passed from the mirrorCtrl
        """
        # print "%s._userCmdCallback(userCmd=%r)" % (self, userCmd)
        if not self._inDevCmdCallback:
            if userCmd.isDone:
                self.clearAll()
            elif userCmd.isFailing:
                # use a timer so cancelling devCmdState sets userCmd state
                # self.clearAll()
                Timer(0, self.clearAll)

    def _devCmdCallback(self, devCmd):
        """Device command callback

        @param[in] devCmd: the device command, passed via callback

        startDevCmd always assigns this as the callback, which then calls and clears any user-specified callback.
        """
        # print "%s._devCmdCallback(devCmd=%r); self.userCmd=%r" % (self, devCmd, self.userCmd)
        self._inDevCmdCallback = True
        try:
            if not devCmd.isDone:
                return

            if not devCmd.didFail and self.nextDevCmdCall is not None:
                # succeeded and more code to execute
                nextDevCmdCall, self.nextDevCmdCall = self.nextDevCmdCall, None
                Timer(0, nextDevCmdCall)
                return

            if not self.userCmd.isDone:
                self.userCmd.setState(devCmd.state)
            self.clearAll() # dev cmd done one way or the other
        finally:
            self._inDevCmdCallback = False

    def clearAll(self):
        """If a device command is currently running, kill it. Put galilDevice in a state to receive new commands.
        """
        # print "%s.clearAll()\n    self.currDevCmd=%r\n    self.userCmd=%r" % (self, self.currDevCmd, self.userCmd)
        self.nextDevCmdCall = None #unnecessary?
        if not self.currDevCmd.isDone:
            # print 'DEV CMD Cancelled via clear all'
            # send an ST and set command state to cancelled
            # raise RuntimeError("clearAll called in galilDevice, but currDevCmd not done: %r"%self.currDevCmd)
            # if self.conn.isConnected:
            #     cmdStr = "ST; XQ#STOP"
            #     # cmdStr = "ST;"
            #     log.info("%s writing %r in clearAll before cancelling currently executing device command %s" % (self, cmdStr, self.currDevCmd.cmdStr))
            #     self.conn.writeLine(cmdStr)
            # self.timer.start(0, self.currDevCmd.setState, self.currDevCmd.Cancelled, textMsg="Device Command Cancelled via clearAll() in galilDevice")
            self.currDevCmd.setState(self.currDevCmd.Cancelled, textMsg="Device Command Cancelled via clearAll() in galilDevice")

        self.parsedKeyList = []
        self.status.iter = 0
        self.status.homing = [0]*self.nAct
        self.status.moving = 0
        self.status.maxDuration = 0
        self.status.duration.reset()

    def cmdHome(self, userCmd, axisList):
        """Home the specified actuators

        @param[in] userCmd: a twistedActor UserCmd associated with the home command
        @param[in] axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
         """
        if not axisList:
            axisList = self.validAxisList
        else:
            axisList = [axis.upper() for axis in axisList]
        # compute new value of status.homing (an array of nAct 0s and 1s)
        # and verify that all the requested axes are valid
        newHoming = [0] * self.nAct
        badAxisList = list()
        for axis in axisList:
            ind = self.axisIndexDict.get(axis)
            if ind is None:
                badAxisList.append(axis)
            else:
                newHoming[ind] = 1
        if badAxisList:
            userCmd.setState(userCmd.Failed, textMsg="Invalid axes %s not in %s" % (badAxisList, self.validAxisList))
            return
            #raise CommandError("Invalid axes %s not in %s" % (badAxisList, self.validAxisList))

        cmdStr = self.formatGalilCommand(
            valueList = [doHome if doHome else None for doHome in newHoming],
            cmd = "XQ #HOME",
        )
        self.runCommand(userCmd, galilCmdStr=cmdStr)
        if self.currDevCmd.state == self.currDevCmd.Running:
            self.status.homing = newHoming
            self.writeToUsers(">", "Text = \"Homing Actuators: %s\"" % (", ".join(str(v) for v in axisList)), cmd=userCmd)
            updateStr = self.status._getKeyValStr(["homing"])
            self.writeToUsers("i", updateStr, cmd=userCmd)
            self.status.maxDuration = 0
            self.status.duration.startTimer()
            userCmd.addCallback(self.sendState)
            self.sendState(cmd=userCmd)
            # self.writeToUsers("i", self.status.currentStatus(), cmd=userCmd)
        else:
            # command failed for some reason, do nothing, should clean itself up
            pass

    def cmdMove(self, userCmd, orient):
        """Accepts an orientation then commands the move.

        @param[in] userCmd: a twistedActor UserCmd object associated with this move command
        @param[in] orient: an orientation.

        Subsequent moves are commanded until an acceptable orientation is reached (within errors).
        Cmd not tied to state of devCmd, because of subsequent moves.
        """
        # enable iteration for mirrors with encoders
        # (user) commanded orient --> mount
        mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient = True, adjustOrient = True)
        desEncMount = self.mirror.encoderMountFromOrient(adjOrient, adjustOrient = False)
        adjOrient = numpy.asarray(adjOrient, dtype=float)
        mount = numpy.asarray(mount, dtype=float)
        # check limits of travel
        for mt, link in itertools.izip(mount, self.mirror.actuatorList):
            if not (link.minMount <= mt <= link.maxMount):
                userCmd.setState(userCmd.Failed, "Commanded orientation %s violates mount limits" % str(orient))
                return

        # apply the current offset (doing this for small moves avoids unwanted mirror motion
        # for tiny corrections; doing it for large moves doesn't seem to hurt and simplifies the code).
        #self.writeToUsers("i", "Text=\"Automatically applying previous offset to mirror move.\"", cmd=userCmd)
        statusStr = self.status._getKeyValStr(["netMountOffset"])
        self.writeToUsers('i', statusStr, cmd=userCmd)

        # format Galil command
        cmdMoveStr = self.formatGalilCommand(valueList=mount+ numpy.asarray(self.status.netMountOffset, dtype=float), cmd="XQ #MOVE")
        self.runCommand(userCmd, galilCmdStr=cmdMoveStr, nextDevCmdCall=self._moveIter)
        if self.currDevCmd.state == self.currDevCmd.Running:
            self.status.moving = 1.
            self.status.modelMount = mount[:] # this will not change upon iteration
            self.status.cmdMount = mount[:] + numpy.asarray(self.status.netMountOffset, dtype=float) # this will change upon iteration
            self.status.desOrient = adjOrient[:] # initial guess for fitter
            self.status.desEncMount = desEncMount
            if self.mirror.hasEncoders:
                self.status.iter = 1
            self.status.desOrientAge.startTimer()
            self.status.maxDuration = 0
            self.status.duration.startTimer()
            userCmd.addCallback(self.sendState)
            self.sendState(cmd=userCmd)
            # self.writeToUsers("i", self.status.currentStatus(), cmd=userCmd)
            statusStr = self.status._getKeyValStr(["desOrient", "cmdMount", "desOrientAge", "desEncMount", "modelMount", "maxIter"])
            self.writeToUsers('i', statusStr, cmd=userCmd)
        else:
            # dev command not running for some reason, must have failed
            pass

    def cmdReset(self, userCmd):
        """Reset the Galil to its power-on state. All axes will have to be re-homed. Stop is gentler!

        @param[in] userCmd: a twistedActor UserCmd

        Send 'RS' to the Galil, causing it to reset to power-up state,
        """
        self.runCommand(userCmd, galilCmdStr="RS", forceKill=True)
        # this command doesn't reply with an OK.  Explicitly set the device command to be done
        # self.currDevCmd.setState(self.currDevCmd.Done)
        Timer(0., self.currDevCmd.setState, self.currDevCmd.Done)

    def cmdStop(self, userCmd, getStatus=False):
        """Stop the Galil.

        @param[in] userCmd: a twistedActor UserCmd

        Send 'ST;XQ#STOP' to the Galil, causing it to stop all threads,
        """
        if getStatus:
            self.runCommand(userCmd, galilCmdStr="ST;XQ#STATUS", forceKill=True)
        else:
            self.runCommand(userCmd, galilCmdStr="ST;XQ#STOP", forceKill=True)

    def cmdCachedStatus(self, userCmd):
        """Return a cached status, don't ask the galil for a fresh one

        @param[in] userCmd: a twistedActor UserCmd
        """
        self.writeToUsers("w", "Text=\"Galil is busy executing: %s, showing cached status\"" % self.currDevCmd.cmdStr, cmd = userCmd)
        statusStr = self.status._getKeyValStr([
            # "maxDuration",
            # "duration",
            "orient",
            "desOrient",
            "desOrientAge",
            "actMount",
            "cmdMount",
            "desEncMount",
            "modelMount",
            # "iter",
            "maxIter",
            "homing",
            "axisHomed",
        ])
        self.writeToUsers("i", statusStr, cmd=userCmd)
        self.sendState(cmd=userCmd)
        # self.writeToUsers("i", self.status.currentStatus(), cmd=userCmd)
        userCmd.setState(userCmd.Done)

    def cmdStatus(self, userCmd):
        """Return the Galil status to the user.

        @param[in] userCmd: a twistedActor UserCmd

        If the Galil is busy then returns cached data.
        """
        self.runCommand(userCmd, galilCmdStr="XQ#STATUS", nextDevCmdCall=self._statusCallback)

    def cmdParams(self, userCmd):
        """Show Galil parameters

        @param[in] userCmd: a twistedActor UserCmd
        """
        self.runCommand(userCmd, galilCmdStr="XQ#SHOWPAR")


    def _moveIter(self):
        """A move device command ended; decide whether further move iterations are required and act accordingly.
        """
        if self.userCmd.isDone or self.userCmd.isFailing:
            log.info("_moveIter early return because self.userCmd.isDone!")
            return
        # check if we got all expected information from Galil...
        if not ('max sec for move' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Move time estimates were not received from move\"", cmd=self.userCmdOrNone)
        if not('target position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Target actuator positions not received from move\"", cmd=self.userCmdOrNone)
        if not('final position' in self.parsedKeyList):
            # final actuator positions are needed for subsequent moves...better fail the cmd
            if not self.currDevCmd.isDone:
                self.currDevCmd.setState(self.currDevCmd.Failed, textMsg="Final actuator positions not received from move")
            return

        actErr = [cmd - act for cmd, act in itertools.izip(self.status.modelMount[0:self.nAct], self.status.actMount[0:self.nAct])]
        self.status.mountErr = actErr[:]
        statusStr = self.status._getKeyValStr(["mountErr"])
        self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)

        # error too large to correct?
        if numpy.any(numpy.abs(actErr) > self.mirror.maxCorrList):
            self.currDevCmd.setState(self.currDevCmd.Failed, "Error too large to correct")
            return

        # perform another iteration?
        if numpy.any(numpy.abs(actErr) > self.mirror.minCorrList) and (self.status.iter < self.status.maxIter):
            newCmdActPos = [err*self.CorrectionStrength + prevMount for err, prevMount in itertools.izip(actErr, self.status.cmdMount)]
            self.status.cmdMount = newCmdActPos
            # record the current offset which is total descrepancy between first commanded (naive) mount position
            # and lastly commanded (iterated/converged) mount position
            self.status.netMountOffset = [cmdLast - cmdFirst for cmdFirst, cmdLast in itertools.izip(self.status.modelMount[0:self.nAct], self.status.cmdMount[0:self.nAct])]
            self.status.mountOrient = numpy.asarray(self.mirror.orientFromActuatorMount(newCmdActPos))
            # clear or update the relevant slots before beginning a new device cmd
            self.parsedKeyList = []
            self.status.iter += 1
            self.status.duration.reset() # new timer for new move
            self.userCmd.setTimeLimit(5)
            statusStr = self.status._getKeyValStr(["modelMount", "cmdMount", "mountOrient", "netMountOffset"])
            self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)
            # convert from numpy to simple list for command formatting
            mount = [x for x in newCmdActPos]
            cmdMoveStr = self.formatGalilCommand(valueList=mount, cmd="XQ #MOVE")
            self.status.maxDuration = 0
            self.status.duration.startTimer()
            self.sendState(cmd=self.userCmdOrNone)
            # self.writeToUsers("i", self.status.currentStatus(), cmd=self.userCmdOrNone)
            self.replaceDevCmd(cmdMoveStr, nextDevCmdCall=self._moveIter)
            return
        # done
        #self.status.netMountOffset = [cmdLast - cmdFirst for cmdFirst, cmdLast in itertools.izip(self.status.modelMount[0:self.nAct], self.status.cmdMount[0:self.nAct])]
        self._moveEnd()

    def _moveEnd(self, *args):
        """ Explicitly call dev cmd callback, to set user command state to done
        """
        self.status.moving = 0
        # statusStr = self.status._getKeyValStr(["moving"])
        # self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)
        self.status.maxDuration = 0
        self.status.duration.startTimer()
        self.sendState(cmd=self.userCmdOrNone)
        # self.writeToUsers("i", self.status.currentStatus(), cmd=self.userCmdOrNone)
        self._devCmdCallback(self.currDevCmd)

    def _statusCallback(self):
        """Callback for status command.
        """
        if not ('commanded position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Desired actuator positions not received\"", cmd=self.userCmdOrNone)
        if not ('actual position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Actual actuator positions not received\"", cmd=self.userCmdOrNone)
        if not ('status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Status word not received\"", cmd=self.userCmdOrNone)
        if not ('axis homed' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Homed axis info not received\"", cmd=self.userCmdOrNone)

        # grab cached info that wasn't already sent to the user
        statusStr = self.status._getKeyValStr([
            # "maxDuration",
            # "duration",
            # "iter",
            "maxIter",
            "desOrient",
            "desOrientAge",
            "desEncMount",
            "homing",
            # "moving",
        ])
        self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)
        self.sendState(cmd=self.userCmdOrNone)
        # self.writeToUsers("i", self.status.currentStatus(), cmd=self.userCmdOrNone)
        self._devCmdCallback(self.currDevCmd)

    def formatGalilCommand(self, valueList, cmd, axisPrefix="", valFmt="%0.f", nAxes=None):
        """Format a Galil command.

        Inputs:
        @param[in] valueList: a list of values
        @param[in] cmd: the command (e.g. "XQ #MOVE")
        @param[in] axisPrefix: a string prefixing each axis
        @param[in] valFmt: value format
        @param[in] nAxes: number of axes in command; if None use all axes

        Values that are None are replaced with MAXINT
        If len(valueList) < number of actuators, the extra axes are also set to MAXINT
        """
        valueList = list(valueList) # incase of numpy array
        if nAxes is None:
            nAxes = self.nAct
        elif nAxes > self.nAct:
            raise RuntimeError("nAxes too big (%d > %d)" % (nAxes, self.nAct))
        if len(valueList) > nAxes:
            raise RuntimeError("Too many values (%d > %d)" % (len(valueList), nAxes))

        def formatValue(val):
            if val is None:
                return "MAXINT"
            else:
                return valFmt % (val,)

        # append None for missing values at end
        fullValueList = valueList + [None]*(nAxes - len(valueList))

        argList = ["%s%s=%s" % (axisPrefix, self.validAxisList[ind], formatValue(val)) for ind, val in enumerate(fullValueList)]

        return "%s; %s" % ("; ".join(argList), cmd)


class GalilDevice25Sec(GalilDevice):
    """A Galil Device controller that is specific to the 2.5m Secondary mirror

    note: add in piezo corrections for move, LCSTOP should be set to 1 to disable
    piezo corrections inside the Galil
    """
    def __init__(self,
        mirror,
        host,
        port,
        callFunc=None,
        name=None,
    ):
        """Construct a GalilDevice25Sec

        @param[in] mirror: an instance of mirrorCtrl.MirrorBase
        @param[in] host: host address of Galil controller
        @param[in] port: port of Galil controller
        @param[in] callFunc: function to call when state of device changes;
            it receives one argument: this device.
            Note that callFunc is NOT called when the connection state changes;
            register a callback with "conn" for that.
        @param[in] name: name of device; if None then use mirror.name
        """
        GalilDevice.__init__(self,
            name = name if name else mirror.name,
            mirror = mirror,
            host = host,
            port = port,
            callFunc = callFunc,
        )
        ## this mirror has extra status entries
        self.status.castDict["piezoStatus"] = int
        self.status.castDict["piezoCorr"] = mountCast
        self.status.piezoStatus = numpy.nan
        self.status.piezoCorr = [numpy.nan]*3

    def actOnKey(self, key, dataList, replyStr):
        """An overwritten version from the base class to incorporate 2.5m Specific Keywords.
        Takes a key parsed from self.parseReply, and chooses what to do with the data

        @param[in] key: parsed descriptive string returned by self.parseReply
        @param[in] dataList: list of data associated with key
        @param[in] replyStr: unparsed reply, for error messages
        """
        # look for piezo-specific parameters

#         You get this along with an XQ#SHOWPAR:
#         Check and try to parse these first as piezo keys which will only be
#         present in the piezo-specific actorkeys keyword dictionary.
#
#          01.00 version of M2-specific additions
#         -00001676.4874,  00001676.4874 min, max piezo position (microsteps)
#          00002705 number of steps of piezo position
#          00000001.2400 resolution (microsteps/piezo ctrl bit)


        if key == 'min':
            msgStr = '%s=%s' % (('piezoMinPos', dataList[0]))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)
            return
        elif key == 'max piezo position (microsteps)':
            msgStr = '%s=%s' % (('piezoMaxPos', dataList[0]))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)
            return
        elif key == 'number of steps of piezo position':
            msgStr = '%s=%s' % (('piezoNSteps', dataList[0]))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)
            return
        elif key == 'resolution (microsteps/piezo ctrl bit)':
            msgStr = '%s=%s' % (('piezoResolution', dataList[0]))
            self.writeToUsers("i", msgStr, cmd=self.userCmdOrNone)
            return
        # look for piezo-specific status
        elif key == 'piezo corrections (microsteps)':
            self.status.piezoCorr = [float(num) for num in dataList]
            outStr = "; ".join((
                self.status._getKeyValStr(["piezoCorr"]),
                self.formatAsKeyValStr("piezoStatusWord", dataList),
            ))
            self.writeToUsers("i", outStr, cmd=self.userCmdOrNone)
            return
        elif key == 'piezo status word':
            self.status.piezoStatus = int(dataList[0])
            updateStr = self.status._getKeyValStr(["piezoStatus"])
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)
            return
        else:
            # not piezo-related, try classical approach
            GalilDevice.actOnKey(self, key=key, dataList=dataList, replyStr=replyStr)

    def movePiezos(self):
        """Move piezo actuators to attempt to correct residual error

        Only axes A-C have piezo actuators.
        """
        actErr = self.status.modelMount[:3] - self.status.actMount[:3] # round?
        prefix = 'LDESPOS'
        axes = ['A', 'B', 'C']
        argList = ["%s%s=%s" % (prefix, axes[ind], int(round(val))) for ind, val in enumerate(actErr)]
        cmdStr = "%s; %s" % ("; ".join(argList), "XQ #LMOVE")
        #cmdStr = self.formatGalilCommand(actErr, "XQ #LMOVE", axisPrefix="LDESPOS", nAxes=3)
        self.replaceDevCmd(cmdStr, nextDevCmdCall=self._piezoMoveCallback)

    def _moveEnd(self, *args):
        """Overwritten from base class, to allow for a piezo move command after all the
        coarse moves have been finished.

        @param[in] *args: passed automatically due to twistedActor callback framework
        """
        self.movePiezos()

    def _piezoMoveCallback(self, devCmd=None):
        """Called when the piezos are finished moving

        @param[in] devCmd: passed via callback
        """
        if not self.currDevCmd.isDone:
            return

        if not('commanded position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Commanded actuator positions not received from piezo move\"", cmd=self.userCmdOrNone)
        if not('actual position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Actual actuator positions not received from piezo move\"", cmd=self.userCmdOrNone)
        if not('status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Status word not received from piezo move\"", cmd=self.userCmdOrNone)
        if not('piezo status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Piezo status word not received from piezo move\"", cmd=self.userCmdOrNone)
        if not('piezo corrections (microsteps)' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Piezo corrections not received from piezo move\"", cmd=self.userCmdOrNone)
        if self.currDevCmd.didFail:
            pass
            # self.userCmd should have been automatically failed
            # self.currDevCmd.setState(self.currDevCmd.Failed, "Piezo correction failed: %s" % (self.currDevCmd._textMsg,))
        else:
            self._devCmdCallback(self.currDevCmd) # will set self.userCmd to done
