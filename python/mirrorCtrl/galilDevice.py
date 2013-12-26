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
import math
import re
from .const import convOrient2UMArcsec, MMPerMicron, RadPerArcSec

import numpy
from RO.StringUtil import quoteStr, strFromException
from RO.SeqUtil import asSequence
from twistedActor import TCPDevice, CommandError, UserCmd
from twisted.internet import reactor

__all__ = ["GalilDevice", "GalilDevice25Sec"]

## a blank userCmd that is already set as done
NullUserCmd = UserCmd(userID=0, cmdStr="") 
NullUserCmd.setState(UserCmd.Done)

## Stop command for direct galil communication
GalCancelCmd = 'ST'
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
    ",".join([str(x) for x in strArray])

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
        return "%.2f"%(time.time() - self.initTime)


class GalilStatus(object):
    """A container for holding the status of the Galil """
    def __init__(self, device):
        """Construct a GalilStatus

        @param[in] device: mirror device
        """
        self.mirror = device.mirror
        ## number of actuators
        self.nAct = len(self.mirror.actuatorList)

        # all the Status keyword/value pairs we will cache, and their initial values
        self.maxDuration = numpy.nan
        self.duration = GalilTimer()
        ## actuator mount determined from a measured orientation
        self.actMount = numpy.asarray([numpy.nan]*self.nAct) # get rid of?
        ## encoder mount reported from the galil
        self.encMount = numpy.asarray([numpy.nan]*self.nAct)
        ## the user commanded mount position
        self.cmdMount = numpy.asarray([numpy.nan]*self.nAct)
        ## subsequent commanded mount positions determined during iterations
        self.cmdMountIter = numpy.asarray([numpy.nan]*self.nAct)
        ## offset applied to a previously commanded mount, a new mountErr is determined each iteration
        self.mountErr = numpy.asarray([0.]*self.nAct) 
        ## last computed total offset between the first commanded mount position and the last commanded mount position after iteration
        self.mountOffset = numpy.asarray([0.]*self.nAct) 
        ## measured orientation (based on encoder lengths)
        self.orient = numpy.asarray([numpy.nan]*6) # get rid of?
        ## orientation based on commanded actuator positions
        self.mountOrient = numpy.asarray([numpy.nan]*6)
        ## desired orientation, user specified, but adjusted for induced motions due to fixed links.
        self.desOrient = numpy.asarray([numpy.nan]*6)
        ## age of desired orientation
        self.desOrientAge = GalilTimer()
        ## current move iteration 
        self.iter = numpy.nan
        ## max number allowed for move iterations
        self.maxIter = device.MaxIter if self.mirror.hasEncoders else 1
        ## status bits
        self.status = numpy.asarray([numpy.nan]*self.nAct)
        ## acutators which are currently homing
        self.homing = ["?"]*self.nAct
        ## actuators which are currently homed
        self.axisHomed = ["?"]*self.nAct
        ## dictionary containing casting strageties to output current gailil status/state info
        self.castDict = {
            "nAct": int,
            "maxDuration": floatCast,
            "duration": self.duration.getTime,
            "actMount": mountCast,
            "encMount": mountCast,
            "cmdMount": mountCast,
            "cmdMountIter": mountCast,
            "mountErr": mountCast,
            "mountOffset": mountCast,
            "orient": orientCast,
            "desOrient": orientCast,
            "mountOrient": orientCast,
            "desOrientAge": self.desOrientAge.getTime,
            "iter": intOrNan,
            "maxIter": intOrNan,
            "status": statusCast,
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


class GalilDevice(TCPDevice):
    """The Galil Device Object

    To do: enforce devCmd time limits.
    Do that locally or have DeviceCommand do it???
    The trick for the latter is knowing when to start the timer,
    but doing so when the command begins is probably sufficient,
    or when the state is Running.
    """
    ## one second to wait after sending an st
    STWaitTime = 1. 
    ## wait two seconds after sending an rs
    RSWaitTime = 2. 
    ## initial timeout to use for every device command
    ## the following are thresholds (large moves) beyond which automatic (previously computed)
    ## move offsets should not be used
    DevCmdTimeout = 2. 
    ## A big piston , indicating a local saved offset should not be applied
    LargePiston = 500. * MMPerMicron
    ## A big translation, indicating a local saved offset should not be applied
    LargeTranslation = 100. * MMPerMicron
    ## A big tilt, indicating a local saved offset should not be applied
    LargeTilt = 10. * RadPerArcSec 
    ## scale the determined move offset correction by this much, eg go only 90%
    CorrectionStrength = 0.9 
    def __init__(self, mirror, host, port, maxIter = 5, callFunc = None):
        """Construct a GalilDevice

        @param[in] mirror: an instance of mirrorCtrl.MirrorBase
        @param[in] host: host address of Galil controller
        @param[in] port: port of Galil controller
        @param[in] maxIter: the maximum number of iterations to refine the mirror position
        @param[in] callFunc: function to call when state of device changes;
            it receives one argument: this device.
            Note that callFunc is NOT called when the connection state changes;
            register a callback with "conn" for that.
        """
        self.MaxIter = maxIter
        self.mirror = mirror
        TCPDevice.__init__(self,
            name = 'galil',
            host = host,
            port = port,
            callFunc = callFunc,
            cmdInfo = (),
        )
        ## currently executing (or last completed) device command
        self.currDevCmd = self.cmdClass("")
        self.currDevCmd.setState(self.currDevCmd.Done)
        ## currently executing (or last completed) user command
        self.userCmd = NullUserCmd
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

    def init(self, userCmd=None, timeLim=None):
        """Initialize Galil
        
        Called on disconnection
        """
        self.cmdStop(userCmd=userCmd)

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
        #print "parseReply(replyStr=%r)" % (replyStr,)
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

        elif TimeEstRegEx.match(key):
            # contains information about estimated execution time
            # update status and notify users
            dataList = numpy.asarray(dataList, dtype=float)
            maxTime = numpy.max(dataList) # get time for longest move
            self.status.maxDuration = maxTime
            updateStr = self.status._getKeyValStr(["maxDuration"])
            # append text describing time for what
            updateStr += '; Text=%s' % (quoteStr(key),)
            self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)
            # adjust time limits
            if not self.currDevCmd.isDone:
                self.currDevCmd.setTimeLimit(maxTime + 2)
            if not self.userCmd.isDone:
                self.userCmd.setTimeLimit(maxTime + 5)

        elif ('commanded position' == key) or ('target position' == key):
            # cmdMount must not be updated for actuator error determination
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
                self.status.actMount = actMount
            updateStr = self.status._getKeyValStr(["orient", "actMount"])
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
        self.logMsg('Galil Reply(%s)' % (replyStr,))
        replyStr = replyStr.replace(":", "").strip(' ;\r\n\x01\x03\x18\x00')
        #print "handleReply(replyStr=%r)" % (replyStr,)
        if self.currDevCmd.isDone:
            # ignore unsolicited input
            return
        if self.currDevCmd.state == self.currDevCmd.Cancelling:
            # wait for cancel echo before failing the cmd
            if GalCancelCmd == replyStr:
                # the echo of the cancel string sent to the galil has returned.
                self.currDevCmd.setState(self.currDevCmd.Cancelled)
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
        if CmdEchoRegEx.search(replyStr) or AxisEchoRegEx.search(replyStr):
            # this is just the command echo, ignore it
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

    def setCurrUserCmd(self, userCmd):
        """Set self.userCmd
        
        @param[in] userCmd: new userCmd, or None if none
            (in which case a blank "done" command is used)
        
        @raise RuntimeError if the existing userCmd and is not done
        """
        if not self.userCmd.isDone:
            raise RuntimeError('User command collision! Cannot replace userCmd unless previous is done!')
        # add a callback to cancel device cmd if the userCmd was cancelled...
        def cancelDev(cbUserCmd):
            if (cbUserCmd.state == cbUserCmd.Cancelling):
                self.currDevCmd.setState(self.currDevCmd.Cancelling)
                self.conn.writeLine(GalCancelCmd)

        if userCmd is None:
            userCmd = NullUserCmd
        else:
            userCmd.addCallback(cancelDev)
        self.userCmd = userCmd

    def cmdHome(self, axisList, userCmd):
        """Home the specified actuators

        @param[in] axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        @param[in] userCmd: a twistedActor UserCmd associated with the home command
        """
        self.setCurrUserCmd(userCmd)
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
            raise CommandError("Invalid axes %s not in %s" % (badAxisList, self.validAxisList))

        cmdStr = self.formatGalilCommand(
            valueList = [doHome if doHome else None for doHome in newHoming],
            cmd = "XQ #HOME",
        )
        self.status.homing = newHoming

        self.writeToUsers(">", "Text = \"Homing Actuators: %s\"" % (", ".join(str(v) for v in axisList)), cmd=self.userCmdOrNone)
        updateStr = self.status._getKeyValStr(["homing"])
        self.writeToUsers("i", updateStr, cmd=self.userCmdOrNone)
        self.startDevCmd(cmdStr, errFunc=self.failStop)

    def cmdMove(self, orient, userCmd):
        """Accepts an orientation then commands the move.

        @param[in] orient: an orientation.
        @param[in] userCmd: a twistedActor UserCmd object associated with this move command 

        Subsequent moves are commanded until an acceptable orientation is reached (within errors).
        Cmd not tied to state of devCmd, because of subsequent moves.
        """
        self.setCurrUserCmd(userCmd)
        # enable iteration for mirrors with encoders
        if self.mirror.hasEncoders:
            self.status.iter = 1
        # (user) commanded orient --> mount
        mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient=True)
        adjOrient = numpy.asarray(adjOrient, dtype=float)
        mount = numpy.asarray(mount, dtype=float)
        self.status.cmdMount = mount[:] # this will not change upon iteration
        # determine if automatic offset should be applied (is this a small move?)
        # self.status.desOrient will not be finite (numpy.nans) upon startup
        addOffset = False
        if numpy.isfinite(numpy.sum(self.status.desOrient)):
            orientDiff = numpy.abs(adjOrient-self.status.desOrient)[:5] #don't care about z-rot
            bigOrient = numpy.asarray([self.LargePiston, self.LargeTilt, self.LargeTilt, self.LargeTranslation, self.LargeTranslation], dtype=float)
            addOffset = numpy.all(numpy.all(orientDiff/bigOrient < 1)) # small move, add offset
        if addOffset:
            # be sure that a previous desOrient is defined (not nans)
            mount = mount + numpy.asarray(self.status.mountOffset, dtype=float)
            self.writeToUsers("i", "Text=\"Automatically applying previous offset to mirror move.\"")
            statusStr = self.status._getKeyValStr(["mountOffset"])
            self.writeToUsers('i', statusStr, cmd=self.userCmdOrNone)
        self.status.cmdMountIter = mount[:] # this will change upon iteration
        self.status.desOrient = adjOrient[:] # initial guess for fitter
        # format Galil command
        cmdMoveStr = self.formatGalilCommand(valueList=mount, cmd="XQ #MOVE")
        self.status.desOrientAge.startTimer()
        statusStr = self.status._getKeyValStr(["desOrient", "desOrientAge", "cmdMount", "maxIter", "iter"])
        self.writeToUsers('i', statusStr, cmd=self.userCmdOrNone)
        self.startDevCmd(cmdMoveStr, callFunc=self._moveIter, errFunc=self.failStop)#timeLim=0.03, 


    def cmdReset(self, userCmd):
        """Reset the Galil to its power-on state. All axes will have to be re-homed. Stop is gentler!

        @param[in] userCmd: a twistedActor UserCmd

        Send 'RS' to the Galil, causing it to reset to power-up state,
        wait a few seconds,
        send XQ#STOP to make sure it is fully reset,
        then send XQ#STATUS to report current state.
        """
        #self.startUserCmd(userCmd, doCancel=True, timeLim=10)
        self.setCurrUserCmd(userCmd)
        self.conn.writeLine('RS')
        reactor.callLater(self.RSWaitTime, self.sendStop) # wait 3 seconds then command stop, then status

    def cmdStop(self, userCmd):
        """Stop the Galil.

        @param[in] userCmd: a twistedActor UserCmd

        Send 'ST' to the Galil, causing it to stop all threads,
        wait a short time,
        send XQ#STOP to make sure it is fully reset,
        then send XQ#STATUS to report current state.
        """
        self.setCurrUserCmd(userCmd)
        #self.conn.writeLine('ST')
        #reactor.callLater(1, self.sendStop) # wait 1 second then command stop, then status
        self.startDevCmd("XQ#STOP", callFunc=self.sendStatus)

    def cmdCachedStatus(self, userCmd):
        """Return a cached status, don't ask the galil for a fresh one

        @param[in] userCmd: a twistedActor UserCmd
        """
        self.writeToUsers("w", "Text=\"Galil is busy executing: %s, showing cached status\"" % self.currDevCmd.cmdStr, cmd = userCmd)
        statusStr = self.status._getKeyValStr([
            "maxDuration",
            "duration",
            "actMount",
            "cmdMount",
            "orient",
            "desOrient",
            "desOrientAge",
            "iter",
            "maxIter",
            "homing",
            "axisHomed",
        ])
        self.writeToUsers("i", statusStr, cmd=userCmd)
        userCmd.setState(userCmd.Done)

    def cmdStatus(self, userCmd):
        """Return the Galil status to the user.

        @param[in] userCmd: a twistedActor UserCmd

        If the Galil is busy then returns cached data.
        """
        self.setCurrUserCmd(userCmd)
        self.startDevCmd("XQ #STATUS", callFunc = self._statusCallback)

    def cmdParams(self, userCmd):
        """Show Galil parameters

        @param[in] userCmd: a twistedActor UserCmd
        """
        #self.startUserCmd(userCmd)
        self.setCurrUserCmd(userCmd)
        self.startDevCmd("XQ #SHOWPAR")

    def sendStop(self, callFunc=None):
        """Send XQ#STOP, then execute callFunc

        @param[in] callFunc: callable, will be a callback set on a device command
        """
        if callFunc == None:
            callFunc = self.sendStatus
        self.startDevCmd("XQ#STOP", callFunc=callFunc)

    def sendStatus(self):
        """Send device command XQ#STATUS"""
        self.startDevCmd("XQ#STATUS")

    def failStop(self):
        """Send device command XQ#STOP, then XQ#STATUS, and afterwards fail the user command"""
        def failStatus():
            self.startDevCmd("XQ#STATUS", callFunc = self._failUserCmd)
        self.startDevCmd("XQ#STOP", callFunc=failStatus)

    def startDevCmd(self, cmdStr, callFunc=None, errFunc=None):
        """Start a new device command, replacing self.currDevCmd

        @param[in] cmdStr: command to send to the Galil
        @param[in] callFunc: function to call when the command finishes; receives no arguments
            (note: _devCmdCallback is called by the device command and is responsible
            for calling callFunc, which is stored in _userCmdNextStep)
        @param[in] errFunc: function to call if device command fails (eg, gailil sends a "?") before
            setting userCmd to failed. Intended primarily for a status query prior to user
            command termination.
        """
        # print "startDevCmd(cmdStr=%s, timeLimt=%s, callFunc=%s)" % (cmdStr, timeLim, callFunc)
        # print "self.userCmd=%r" % (self.userCmd,)
        # print "self.devCmd=%r" % (self.currDevCmd,)
        if not self.currDevCmd.isDone:
            # this should never happen, but...just in case
            raise RuntimeError("Cannot start new device command: %s , %s is currently running" % (cmdStr, self.currDevCmd,))

        self._userCmdNextStep = callFunc
        self._userCmdCatchFail = errFunc
        devCmd = self.cmdClass(cmdStr, timeLim = self.DevCmdTimeout, callFunc=self._devCmdCallback)
        self.currDevCmd = devCmd
        self.parsedKeyList = []
        try:
            self.logMsg("DevCmd(%s)" % (devCmd.cmdStr,))
            self.conn.writeLine(devCmd.cmdStr)
            devCmd.setState(devCmd.Running)
        except Exception, e:
            devCmd.setState(devCmd.Failed, textMsg=strFromException(e))
            self._cleanup()

    def _failUserCmd(self):
        """Simply fail the current user command
        """
        self._cleanup()
        self._userCmdNextStep = None
        if not self.userCmd.isDone:
            self.userCmd.setState(self.userCmd.Failed,
                textMsg="Galil command %s failed" % (self.userCmd.cmdStr,))

    def _cancelUserCmd(self):
        """Simply cancel the current user command
        """
        self._cleanup()
        self._userCmdNextStep = None
        self._userCmdCatchFail = None
        self.userCmd.setState(self.userCmd.Cancelled,
            textMsg="Galil command %s cancelled" % (self.userCmd.cmdStr,))

    def _devCmdCallback(self, dumDevCmd=None):
        """Device command callback

        @param[in] dumDevCmd: dummy argument, for callback

        startDevCmd always assigns this as the callback, which then calls and clears any user-specified callback.
        """
        # print 'dev cmd state', self.currDevCmd.cmdStr, self.currDevCmd.state, self.currDevCmd._textMsg
        # print "_devCmdCallback(); currDevCmd=%r; _userCmdNextStep=%s" % (self.currDevCmd, self._userCmdNextStep)
        # print "_devCmdCallback(); self.userCmd=%r" % (self.userCmd,)

        if self.currDevCmd.state == self.currDevCmd.Cancelled:
            self.writeToUsers("w", "Text=\"Device Command %s Cancelled: %s\"" % (self.currDevCmd.cmdStr, self.currDevCmd._textMsg,), cmd = self.currDevCmd)
            self._cancelUserCmd()
            return
        if self.currDevCmd.state == self.currDevCmd.Failed:
            self.writeToUsers("w", "Text=\"Device Command %s Failed: %s\"" % (self.currDevCmd.cmdStr, self.currDevCmd._textMsg,), cmd = self.currDevCmd)
            userCmdCatchFail, self._userCmdCatchFail = self._userCmdCatchFail, None
            if userCmdCatchFail:
                # don't do this if cmd was 'cancelled', only if 'failed'
                self.conn.writeLine("ST")
                reactor.callLater(self.STWaitTime, userCmdCatchFail) # I imagine will almost always be self.failStop()
            else:
                self._failUserCmd()
            return

        if not self.currDevCmd.isDone:
            return
        userCmdNextStep, self._userCmdNextStep = self._userCmdNextStep, None
        if userCmdNextStep:
            # start next step of user command
            userCmdNextStep()
        else:
            # nothing more to do; user command must be finished!
            self.userCmd.setState(self.userCmd.Done)
            self._cleanup()
        #print "_devCmdCallback()END; self.userCmd=%r" % (self.userCmd,)

    def _moveIter(self):
        """A move device command ended; decide whether further move iterations are required and act accordingly.
        """
        # check if we got all expected information from Galil...
        if not ('max sec for move' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Move time estimates were not received from move\"", cmd=self.userCmdOrNone)
        if not('target position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Target actuator positions not received from move\"", cmd=self.userCmdOrNone)
        if not('final position' in self.parsedKeyList):
            # final actuator positions are needed for subsequent moves...better fail the cmd
            self.userCmd.setState(self.userCmd.Failed, textMsg="Final actuator positions not received from move")
            return

        actErr = [cmd - act for cmd, act in itertools.izip(self.status.cmdMount[0:self.nAct], self.status.actMount[0:self.nAct])]
        self.status.mountErr = actErr[:]

        # error too large to correct?
        if numpy.any(numpy.abs(actErr) > self.mirror.maxCorrList):
            self.userCmd.setState(self.userCmd.Failed, "Error too large to correct")
            return

        # perform another iteration?
        if numpy.any(numpy.abs(actErr) > self.mirror.minCorrList) and (self.status.iter < self.status.maxIter):
            newCmdActPos = [err*self.CorrectionStrength + prevMount for err, prevMount in itertools.izip(actErr, self.status.cmdMountIter)]
            self.status.cmdMountIter = newCmdActPos
            self.status.mountOrient = numpy.asarray(self.mirror.orientFromActuatorMount(newCmdActPos))
            # clear or update the relevant slots before beginning a new device cmd
            self.parsedKeyList = []
            self.status.iter += 1
            self.status.duration.reset() # new timer for new move
            self.userCmd.setTimeLimit(5)
            statusStr = self.status._getKeyValStr(["cmdMount", "cmdMountIter" ,"iter", "mountErr", "mountOrient"])
            self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)
            # convert from numpy to simple list for command formatting
            mount = [x for x in newCmdActPos]
            cmdMoveStr = self.formatGalilCommand(valueList=mount, cmd="XQ #MOVE")
            self.startDevCmd(cmdMoveStr, callFunc=self._moveIter, errFunc=self.failStop)
            return
        # done
        # record offset which is total descrepancy between first commanded (naive) mount position
        # and lastly commanded (iterated/converged) mount position
        # offset should be added
        self.status.mountOffset = [cmdLast - cmdFirst for cmdFirst, cmdLast in itertools.izip(self.status.cmdMount[0:self.nAct], self.status.cmdMountIter[0:self.nAct])]
        self._moveEnd()

    def _moveEnd(self, *args):
        """The final move device command ended successfully; clean up and set self.userCmd state done.

        Optionally perform any post-move cleanup (such as moving piezos).
        Finally: set self.userCmd state done (or failed, if cleanup failed).

        note: *args for callbackability...
        """
        #if not self.userCmd.isDone:
        self.userCmd.setState(self.userCmd.Done)

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
            "maxDuration",
            "duration",
            "iter",
            "maxIter",
            "desOrient",
            "desOrientAge",
            "homing",
        ])
        self.writeToUsers("i", statusStr, cmd=self.userCmdOrNone)
        self.userCmd.setState(self.userCmd.Done)

    def _cleanup(self):
        """Clean up between device commands
        """
        if not self.currDevCmd.isDone:
            raise RuntimeError('currend dev cmd must be finished before cleanup')
        self.parsedKeyList = []
        self.status.iter = numpy.nan
        self.status.homing = [0]*self.nAct
        self.status.maxDuration = numpy.nan
        self.status.duration.reset()

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
        callFunc = None,
    ):
        """Construct a GalilDevice25Sec

        @param[in] mirror: an instance of mirrorCtrl.MirrorBase
        @param[in] host: host address of Galil controller
        @param[in] port: port of Galil controller
        @param[in] callFunc: function to call when state of device changes;
            it receives one argument: this device.
            Note that callFunc is NOT called when the connection state changes;
            register a callback with "conn" for that.
        """
        GalilDevice.__init__(self,
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
        actErr = self.status.cmdMount[:3] - self.status.actMount[:3] # round?
        prefix = 'LDESPOS'
        axes = ['A', 'B', 'C']
        argList = ["%s%s=%s" % (prefix, axes[ind], int(round(val))) for ind, val in enumerate(actErr)]
        cmdStr = "%s; %s" % ("; ".join(argList), "XQ #LMOVE")
        #cmdStr = self.formatGalilCommand(actErr, "XQ #LMOVE", axisPrefix="LDESPOS", nAxes=3)

        self.startDevCmd(cmdStr, callFunc=self._piezoMoveCallback)

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
            self.userCmd.setState(self.userCmd.Failed, "Piezo correction failed: %s" % (self.currDevCmd._textMsg,))
        else:
            self.userCmd.setState(self.userCmd.Done)
