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

import numpy
from RO.StringUtil import quoteStr, strFromException
from RO.SeqUtil import asSequence
from twistedActor import TCPDevice, CommandError, UserCmd
from twisted.internet import reactor

__all__ = ["GalilDevice", "GalilDevice25Sec"]

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec
# scale orient array by this to convert into user-units
ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                                        MMPerMicron, MMPerMicron, RadPerArcSec], dtype = float)

# RegEx stuff up here to keep these from being needlessly re-compiled...
startsWithNumRegEx = re.compile(r'^-?[0-9]')
# find numbers not preceeded by a '/' or a letter and not followed by a letter
getDataRegEx = re.compile(r'(?<!/|[A-Za-z])[0-9-.]+(?![A-Za-z])')
# params are uppercase
paramRegEx = re.compile(r'^-?[A-Z]')
timeEstRegEx = re.compile(r'^sec +to|^max +sec|^time +for')
okLineRegEx = re.compile(r'^OK$', re.IGNORECASE)

cmdEchoRegEx = re.compile(r'xq *#[a-z]+$', re.IGNORECASE)
axisEchoRegEx = re.compile(r'[A-Z]=-?(\d+)', re.IGNORECASE)

MaxIter = 2

class GalilTimer(object):
    """Measure elapsed time
    """
    def __init__(self):
        """Construct a GalilTimer in the reset state"""
        self.reset()

    def reset(self):
        """Reset (halt) timer; getTime will return nan until startTimer called."""
        self.initTime = numpy.nan

    def startTimer(self):
        """Start the timer"""
        self.initTime = time.time()

    def getTime(self):
        """Return elapsed time since last call to startTimer
        
        Return nan if startTimer not called since construction or last reset.
        """
        return time.time() - self.initTime

class GalilStatus(object):
    """A container for holding the status of the Galil """
    def __init__(self, device):
        """Initialize all keywords in StatusKeys """
        self.mirror = device.mirror
        self.nAct = len(self.mirror.actuatorList)

        # all the Status keyword/value pairs we will cache, and their initial values
        self.maxDuration = numpy.nan
        self.duration = GalilTimer()
#        self.remainExecTime = numpy.nan
        self.actMount = [numpy.nan]*self.nAct # get rid of?
        self.encMount = [numpy.nan]*self.nAct
        self.cmdMount = [numpy.nan]*self.nAct # updated upon iteration
        self.orient = [numpy.nan]*6 # get rid of?
        self.desOrient = [numpy.nan]*6
        self.desOrientAge = GalilTimer()
        self.iter = numpy.nan
        self.maxIter = MaxIter if self.mirror.hasEncoders else 1
        self.status = [numpy.nan]*self.nAct
        self.homing = ["?"]*self.nAct
        self.axisHomed = ["?"]*self.nAct

    def _getKeyValStr(self, keywords):
        """Package and return current keyword value info in status cache

        input:
        - keywords: a list of keywords for which you wish to return a formatted string.


        output:
        - statusStr: string in the correct keword value format to be sent to users
        """
        strList = []
        for keyword in keywords:
            val = getattr(self, keyword)
            if keyword in ['duration', 'desOrientAge']:
                # get current execution time
                val = val.getTime()
#             if keyword == 'remainExecTime':
#                 # get current execution time
#                     val = self.maxDuration - self.duration.getTime()
            if keyword in ['orient', 'desOrient']:
                # convert to user-friendly units
                val = numpy.divide(val, ConvertOrient)
            if type(val) in [list, numpy.ndarray]:
                # val is a list or numpy array, we need to format as comma seperated string
                strVal = ", ".join(str(x) for x in val)
            else:
                strVal = str(val)
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
    def __init__(self, mirror, host, port, callFunc = None):
        """Construct a GalilDevice
        
        Inputs:
        - mirror    an instance of mirrorCtrl.MirrorBase
        - host      host address of Galil controller
        - port      port of Galil controller
        - callFunc  function to call when state of device changes;
                    note that it is NOT called when the connection state changes;
                    register a callback with "conn" for that task.
        """
        self.mirror = mirror
        TCPDevice.__init__(self,
            name = 'galilDevice',
            host = host,
            port = port,
            callFunc = callFunc,
            cmdInfo = (),
        )
        self.currDevCmd = self.cmdClass("")
        self.currDevCmd.setState(self.currDevCmd.Done)
        self.currUserCmd = UserCmd(userID=0, cmdStr="")
        self.currUserCmd.setState(self.currUserCmd.Done)
        self.queuedUserCmd = UserCmd(userID=0, cmdStr="")
        self.queuedUserCmd.setState(self.queuedUserCmd.Done)
        self.parsedKeyList = []
        self.nAct = len(self.mirror.actuatorList)
        self.validAxisList =  ('A', 'B', 'C', 'D', 'E', 'F')[0:self.nAct]
        # dictionary of axis name: index, e.g. A: 0, B: 1..., F: 5
        self.axisIndexDict = dict((axisName, ind) for ind, axisName in enumerate(self.validAxisList))
        self.status = GalilStatus(self)

    def formatAsKeyValStr(self, keyword, value):
        """Format a keyword/value pair string.

        Inputs:
        -keyword: a keyword (string)
        -value: the associated value for the keyword, should always be a list

        Output:
        -outStr: a string in the correct format to be sent to the user
        """
        valStr = ', '.join(str(v) for v in asSequence(value))
        outStr = keyword + '=' + valStr
        return outStr

    def parseLine(self, line):
        """Tries to parse a reply from the Galil and seperate into descriptor-value format.
        I consider any descriptive text following the data a descriptor, there can be multiple
        descriptor and values on a single line.

        output:
        - dataMatched: string array containing data
        - keys: string array, text describing any data

        example lines:
            0000.2,  0362.7,  0000.2,  0000.0,  0000.0 max sec to find reverse limit switch
             0300.1,  0300.1,  0300.1,  0000.0,  0000.0 max sec to find home switch
             0008.2,  0008.2,  0008.2,  0000.0,  0000.0 sec to move away from home switch
            Finding next full step
             041,  006.6 microsteps, sec to find full step
            -000006732,  000014944,  000003741,  999999999,  999999999 position error
             1,  1,  1,  0,  0 axis homed
        """
        #print "parseLine(line=%r)" % (line,)
        # Grab the data in each line:
        # Match only numbers (including decimal pt and negative sign) that are not preceeded by '/'
        # or immediately surrounded by any letters letters.
        # Eg: 'RNGx/2' which I don't want to match
        dataMatched = getDataRegEx.findall(line) # will put non-overlapping data (numbers) in a list
        numVals = len(dataMatched)
        # Grab descriptor info on each line, which follows data.
        # split on whitespace n times, last split will be descriptor info
        textOnly = re.split(r' +', line, maxsplit = numVals)[-1]
        # if there are multiple descriptors on a line, they will be separated by a ','
        # or whitespace following a '?'
        # ....so split the text further
        keys = re.split(r',|(?<=\?) ', textOnly)
        # remove leading and trailing whitespace in each descriptor
        keys = [key.strip() for key in keys]
        # on each incoming data line we expect either
        # (1) 1 descriptor and 1 value for each actuator
        # or
        # (2) n descriptors and n values
        
        # note: commented below because in the case of piezo's len(dataMatched) may != self.nAct
#         goodMatch = [
#             (len(dataMatched) == len(keys)),
#             ((len(keys) == 1) and (len(dataMatched) == self.nAct))
#             ]
#         if not (True in goodMatch):
#             # There is confusion in the amount of descriptors and values in this particular line.
#             # report it but keep going
#             self.writeToUsers("w", "BadGalilReply=%s, \"num descriptors do not match num values.\"" % (quoteStr(line),), cmd=self.currUserCmd)
#             return
        # check to see if descriptor/s are an expected reply
#         knownReply = [(key in self.expectedReplies) for key in keys]
#         if False in knownReply:
#             # a descriptor wasn't recognized as an expected reply for this Galil/mirror
#             # report it but keep going
#             print 'keys: ', keys
#             self.writeToUsers("w","BadGalilReply=%s, \"Descriptor text not recognized.\"" % (quoteStr(line),), cmd=self.currUserCmd)
#             return
        return dataMatched, keys

    def sendGalilParam(self, key, data):
        """Key is determined to be a Galil parameter, send the key and data to the user.
        Keyword for Galil Paramaters are 'GPar<par>' where par is the name from the
        Galil itself.

        key example: -RNGx/2 reverse limits

        """
        param = key.split()[0] # in example: just keep -RNGx/2
        # format as a legal keyword swap out '-' and '/' if present
        if param == '-RNGx/2':
            # skip it
            return
        elif param == 'RNGx/2':
            param = 'HalfRNGx'
        keyword = 'GPar%s' % (param)
        outStr = self.formatAsKeyValStr(keyword, data)
        self.writeToUsers("i", outStr, cmd = self.currUserCmd)

    def actOnKey(self, key, data):
        """Takes a key parsed from self.parseLine, and chooses what to do with the data

        inputs:
        -key: parsed descriptive string from self.parseLine (a list)
        -data: parsed data from self.parseLine (a list)
        """
         #line begins with cap letter (or -cap letter)
        if paramRegEx.match(key):
            # this is a Galil parameter, format it and send it to the user
            self.sendGalilParam(key, data)
            return

        elif 'software version' in key:
            # this is a special parameter case, and can't be treated using sendParam
            msgStr = 'GPar%s=%s' % (('SoftwareVersion', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return

        elif timeEstRegEx.match(key):
            # contains information about estimated execution time
            # update status and notify users
            data = numpy.asarray(data, dtype=float)
            maxTime = numpy.max(data) # get time for longest move
            self.status.maxDuration = maxTime
            updateStr = self.status._getKeyValStr(["maxDuration"])
            # append text describing time for what
            updateStr += '; Text=%s' % (quoteStr(key),)
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            # adjust time limits
            self.currDevCmd.setTimeLimit(maxTime + 2)
            self.currUserCmd.setTimeLimit(maxTime + 5)
            return

        elif ('commanded position' == key) or ('target position' == key):
            # in the case where cmdMount == 999999999 set to NaN
            self.status.cmdMount = [int(x) if int(x) != 999999999 else numpy.nan for x in data]
            #self.status.cmdMount = cmdMount if cmdMount != [999999999]*self.nAct else [numpy.nan]*self.nAct
            updateStr = self.status._getKeyValStr(["cmdMount"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return

        elif ('actual position' == key) or ('final position' == key):
            # measured position (adjusted)
            # account for encoder --> actuator spatial difference
            #self.status.encMount = numpy.asarray(data, dtype=float)
            # in the case where encMount == 999999999 set to NaN
            self.status.encMount = [int(x) if int(x) != 999999999 else numpy.nan for x in data]
            
            #self.status.encMount = encMount if encMount != [999999999]*self.nAct else [numpy.nan]*self.nAct
            updateStr = self.status._getKeyValStr(["encMount"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            # DesOrient may be nans
            if not numpy.isfinite(sum(self.status.encMount)):
                # encoder positons contain NaN(s)
                # Further computations cannot be done
                self.status.orient = [numpy.nan]*6
                self.status.actMount = [numpy.nan]*self.nAct
            else:    
                # initial guess is desOrient, unless nothing is there, in which case start from zero
                initOrient = self.status.desOrient if numpy.isfinite(sum(self.status.desOrient)) else numpy.zeros(6)
                orient = self.mirror.orientFromEncoderMount(self.status.encMount, initOrient)
                actMount = self.mirror.actuatorMountFromOrient(orient)
                actMount = numpy.asarray(actMount, dtype=float)
                # add these new values to status
                self.status.orient = numpy.asarray(orient[:], dtype=float)
                self.status.actMount = actMount            
            updateStr = self.status._getKeyValStr(["orient", "actMount"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return

        elif key == 'axis homed':
            data = [int(num) for num in data]
            self.status.axisHomed = data
            updateStr = self.status._getKeyValStr(["axisHomed"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return

        elif key == 'status word':
            data = [int(num) for num in data]
            self.status.status = data
            updateStr = self.status._getKeyValStr(["status"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return

        else:
            # send the remaining info back to the user
            msgStr = "UnparsedReply=%s, %s" % (quoteStr(key), quoteStr(str(data)))
            self.writeToUsers("i", msgStr, cmd=self.currUserCmd)
            return

    def handleReply(self, replyStr):
        """Handle a line of output from the device. Called whenever the device outputs a new line of data.

        Inputs:
        - replyStr  the reply, minus any terminating \n

        Tasks include:
        - Parse the reply
        - Manage the pending commands
        - Output data to users
        - Parse status to update the model parameters
        - If a command has finished, call the appropriate command callback
        """
        self.logMsg('Galil Reply(%s)' % (replyStr,))
        #log.msg('Galil Reply: ' + replyStr)
        #print 'Galil Reply: ' + replyStr
        #print "handleReply(replyStr=%r)" % (replyStr,)
        if self.currDevCmd.isDone:
            # ignore unsolicited input
            return
        
        #replyStr = unicode(replyStr, errors='ignore')
        #replyStr = replyStr.encode("ascii", errors = "ignore")        
        replyStr = replyStr.replace(":", "").strip(' ;\r\n\x01\x03\x18\x00')
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
        if okLineRegEx.match(replyStr):
            # command finished
            state, foo, bar = self.currDevCmd.fullState
            if state == self.currDevCmd.Failing:
                # got the OK, now fail the command
                self.currDevCmd.setState(self.currDevCmd.Failed)
            else:
                self.currDevCmd.setState(self.currDevCmd.Done)
            return
        cmdEcho = cmdEchoRegEx.search(replyStr) or axisEchoRegEx.search(replyStr)
        if cmdEcho:
            # this is just the command echo, ignore it
            return
        if not startsWithNumRegEx.match(replyStr):
            # line doesn't begin with a data value, eg 'Finding next full step'
            # show it to the user and return
            self.writeToUsers("i", "UnparsedReply=\"%s\"" % (replyStr,), cmd = self.currUserCmd)
            return
        # if we made it this far, separate the data from the descriptive text
        parsedLine = self.parseLine(replyStr)
        if not parsedLine:
            # parse didn't work, but keep going
            return
        else:
            data = parsedLine[0]
            description = parsedLine[1]

        if len(description) == 1:
            key = description[0]
            self.actOnKey(key, data)
            self.parsedKeyList.append(key)
        else:
            for key, dat in itertools.izip(description, data):
                    # description and data are always lists..so iterable
                    # even if they contain only one value
                    self.actOnKey(key, dat)
                    self.parsedKeyList.append(key)

    def cmdHome(self, axisList, userCmd):
        """Home the specified actuators

        Inputs:
        - axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        """
        def doHome(cmd=None, axisList=None):
            """cmd arg in case issued as a callback to a command
            """
            if (cmd!=None):
                if not cmd.isDone:
                    return
                else:
                    # set quedUserCmd to current user cmd
                    # get the axisList which was saved in the queued cmd
                    axisList = self.queuedUserCmd.axisList
                    self.currUserCmd=self.queuedUserCmd
                    # incase the previously queued command was superceeded...
                    if self.currUserCmd.isDone:
                        return
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
            
            self.writeToUsers(">", "Text = \"Homing Actuators: %s\"" % (", ".join(str(v) for v in axisList)), cmd = userCmd)
            updateStr = self.status._getKeyValStr(["homing"])
            self.writeToUsers("i", updateStr, cmd=userCmd)
            self.startDevCmd(cmdStr, errFunc=self.failStop)

        self.startUserCmd(userCmd)
        if userCmd.isDone:
            # command was rejected, device was busy, return empty handed
            return
        elif userCmd.state == userCmd.Ready:
            # command not running, not done, must have been queued because a status is running.
            # add callback to current command
            # hack?
            self.queuedUserCmd.axisList = axisList
            self.currUserCmd.addCallback(doHome)
        else:
            doHome(axisList=axisList)
        
    def cmdMove(self, orient, userCmd):
        """Accepts an orientation then commands the move.

        Subsequent moves are commanded until an acceptable orientation is reached (within errors).
        Cmd not tied to state of devCmd, because of subsequent moves.
        """
        def doMove(cmd=None):
            """command in case issued as callback
            """
            if (cmd!=None):
                if not cmd.isDone:
                    return
                else:
                    # set quedUserCmd to current user cmd
                    self.currUserCmd=self.queuedUserCmd
                    # incase the previously queued command was superceeded...
                    if self.currUserCmd.isDone:
                        return
            # enable iteration for mirrors with encoders
            if self.mirror.hasEncoders:
                self.status.iter = 1
            # (user) commanded orient --> mount
            mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient=True)
            self.status.cmdMount = numpy.asarray(mount, dtype=float) # this will change upon iteration
            self.status.desOrient = numpy.asarray(adjOrient, dtype=float) # initial guess for fitter
            # format Galil command
            cmdMoveStr = self.formatGalilCommand(valueList=mount, cmd="XQ #MOVE")
            self.status.desOrientAge.startTimer()
            statusStr = self.status._getKeyValStr(["desOrient", "desOrientAge", "cmdMount", "maxIter", "iter"])
            self.writeToUsers('i', statusStr, cmd=userCmd)
            self.startDevCmd(cmdMoveStr, callFunc=self._moveIter, errFunc=self.failStop)
        
        if 'move' in self.currDevCmd.cmdStr.lower():
            self.startUserCmd(userCmd, doCancel=True, timeLim=5)
            self.conn.writeLine('ST')
            # send ST, then XQ# Stop, and finally (when finished) send the new move
            reactor.callLater(1, self.sendStop, doMove) # wait 1 second then command stop, then status   
        else:    
            self.startUserCmd(userCmd, doCancel=False, timeLim=5)
            if userCmd.isDone:
                # command was rejected, device was busy, return empty handed
                return
            elif userCmd.state == userCmd.Ready:
                # command not running, not done, must have been queued because a status is running.
                # add callback to current command
                self.currUserCmd.addCallback(doMove)
            else:
                doMove()


    def cmdReset(self, userCmd):
        """Reset the Galil to its power-on state. All axes will have to be re-homed. Stop is gentler!
        
        Send 'RS' to the Galil, causing it to reset to power-up state,
        wait a few seconds,
        send XQ#STOP to make sure it is fully reset,
        then send XQ#STATUS to report current state.
        """
        self.startUserCmd(userCmd, doCancel=True, timeLim=10)
        self.conn.writeLine('RS')
        reactor.callLater(3, self.sendStop) # wait 3 seconds then command stop, then status

    def cmdStop(self, userCmd):
        """Stop the Galil.
        
        Send 'ST' to the Galil, causing it to stop all threads,
        wait a short time,
        send XQ#STOP to make sure it is fully reset,
        then send XQ#STATUS to report current state.
        """
        self.startUserCmd(userCmd, doCancel=True)
        self.conn.writeLine('ST')
        reactor.callLater(1, self.sendStop) # wait 1 second then command stop, then status

    def cmdStatus(self, userCmd):
        """Return the Galil status to the user.
        
        If the Galil is busy then returns cached data.
        """ 
        if not self.currDevCmd.isDone:
            self.writeToUsers("w", "Text=\"Galil is busy, showing cached status\"", cmd = self.currDevCmd)
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
            return

        self.startUserCmd(userCmd, timeLim=5)
        self.startDevCmd("XQ #STATUS", callFunc = self._statusCallback)

    def cmdParams(self, userCmd):
        """Show Galil parameters
        """
        self.startUserCmd(userCmd)
        if userCmd.isDone:
            # command was rejected, device was busy, return empty handed
            return
        self.startDevCmd("XQ #SHOWPAR")
    
    def sendStop(self, callFunc=None):
        """Send XQ#STOP, then execute callFunc"""
        if callFunc == None:
            callFunc = self.sendStatus
        self.startDevCmd("XQ#STOP", callFunc=callFunc)
    
    def sendStatus(self):
        """Send device command XQ#STATUS"""
        self.startDevCmd("XQ#STATUS")
        
    def failStop(self):
        """Send device command XQ#STOP, then XQ#STATUS, and afterwards fail the user Cmd"""
        def failStatus():
            self.startDevCmd("XQ#STATUS", callFunc = self._failUserCmd)       
        self.startDevCmd("XQ#STOP", callFunc=failStatus)

    def startDevCmd(self, cmdStr, timeLim=2, callFunc=None, errFunc=None):
        """Start a new device command, replacing self.currDevCmd
        
        Inputs:
        - cmdStr: command to send to the Galil
        - timeLim: time limit for command, in seconds
        - callFunc: function to call when the command finishes; receives no arguments
            (note: _devCmdCallback is called by the device command and is responsible
            for calling callFunc, which is stored in _userCmdNextStep)
        - errFunc: function to call if device command fails (eg, gailil sends a "?") before 
            setting userCmd to failed. Intended primarily for a status query prior to user 
            command termination.
        """
        #print "startDevCmd(cmdStr=%s, timeLimt=%s, callFunc=%s)" % (cmdStr, timeLim, callFunc)
        #print "self.userCmd=%r" % (self.currUserCmd,)
        if not self.currDevCmd.isDone:
            # this should never happen, but...just in case
            raise RuntimeError("Cannot start new device command: %s , %s is currently running" % (cmdStr, self.currDevCmd,))
        
        self._userCmdNextStep = callFunc
        self._userCmdCatchFail = errFunc
        devCmd = self.cmdClass(cmdStr, timeLim = timeLim, callFunc=self._devCmdCallback)
        self.currDevCmd = devCmd
        self.parsedKeyList = []
        try:
            self.logMsg("DevCmd(%s)" % (devCmd.cmdStr,))
            self.conn.writeLine(devCmd.cmdStr)
            devCmd.setState(devCmd.Running)
        except Exception, e:
            devCmd.setState(devCmd.Failed, textMsg=strFromException(e))
            self._clearDevCmd()
    
    def startUserCmd(self, userCmd, doCancel=False, timeLim=5):
        """Start a new user command
        
        - If a user command is already running then fail the new command or supersede the old one,
            depending on doCancel
        - Replace self.currUserCmd with new user command
        - Set new comamnd's time limit and set its state to running
        
        Inputs:
        - userCmd: new user command
        - doCancel: if False then reject userCmd if busy;
            if True then cancel current userCmd and devCmd
        - timeLim: time limit for userCmd; if None then leave it alone
        """
        if doCancel:
            self._userCmdNextStep = None
            if not self.currUserCmd.isDone:
                self.currUserCmd.setState(self.currUserCmd.Cancelled, "Superseded")
            if not self.queuedUserCmd.isDone:
                self.queuedUserCmd.setState(self.queuedUserCmd.Cancelled, "Superseded")
            if not self.currDevCmd.isDone:
                self.currDevCmd.setState(self.currDevCmd.Cancelled, "Superseded")
        elif ('status' in self.currDevCmd.cmdStr.lower()) and (not self.currDevCmd.isDone):
            # if currently running command is a status, queue this userCmd to run 
            # immediately after status completion
            self.queuedUserCmd = userCmd
            return
        else:
            if not self.currUserCmd.isDone:
                userCmd.setState(userCmd.Failed, "Busy running user command %s" % (self.currUserCmd.cmdStr,))
                return
            if not self.currDevCmd.isDone:
                raise RuntimeError("Bug! A device command is running (%s) but a user command is not" % \
                    (self.currDevCmd.cmdStr,))
        
        self.currUserCmd = userCmd
        if timeLim is not None:
            userCmd.setTimeLimit(timeLim)
        userCmd.setState(userCmd.Running)
        return

    def _failUserCmd(self):
        """Simply fail the current user command
        """
        self._userCmdNextStep = None
        if not self.currUserCmd.isDone:
            self.currUserCmd.setState(self.currUserCmd.Failed,
                textMsg="Galil command %s failed" % (self.currUserCmd.cmdStr,))
                    
    def _devCmdCallback(self, dumDevCmd=None):
        """Device command callback
        
        startDevCmd always assigns this as the callback, which then calls and clears any user-specified callback.
        """
        #print "_devCmdCallback(); currDevCmd=%r; _userCmdNextStep=%s" % (self.currDevCmd, self._userCmdNextStep)
        #print "_devCmdCallback(); self.userCmd=%r" % (self.currUserCmd,)
        if self.currDevCmd.didFail: #includes 'cancelled' state
        #if self.currDevCmd.state == 'failed':
            self._clearDevCmd()
            userCmdCatchFail, self._userCmdCatchFail = self._userCmdCatchFail, None
            if userCmdCatchFail and self.currDevCmd.state == 'failed':
                # don't do this if cmd was 'cancelled', only if 'failed'
                userCmdCatchFail() # I imagine will almost always be self.failStop()
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
            self.currUserCmd.setState(self.currUserCmd.Done)
            self._clearDevCmd()
        #print "_devCmdCallback()END; self.userCmd=%r" % (self.currUserCmd,)

    def _moveIter(self):
        """A move device command ended; decide whether further move iterations are required and act accordingly.
        """
        # check if we got all expected information from Galil...
        if not ('max sec for move' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Move time estimates were not received from move\"", cmd=self.currUserCmd)
        if not('target position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Target actuator positions not received from move\"", cmd=self.currUserCmd)
        if not('final position' in self.parsedKeyList):
            # final actuator positions are needed for subsequent moves...better fail the cmd
            self.currUserCmd.setState(self.currUserCmd.Failed, textMsg="Final actuator positions not received from move")
            return

        actErr = self.status.cmdMount - self.status.actMount

        # error too large to correct?
        if numpy.any(numpy.abs(actErr) > self.mirror.maxCorrList):
            self.currUserCmd.setState(self.currUserCmd.Failed, "Error too large to correct")
            return

        # perform another iteration?
        if numpy.any(numpy.abs(actErr) > self.mirror.minCorrList) and (self.status.iter < self.status.maxIter):
            self.status.cmdMount += actErr
            # clear or update the relevant slots before beginning a new device cmd
            self.parsedKeyList = []
            self.status.iter += 1
            self.status.duration.reset() # new timer for new move
            self.currUserCmd.setTimeLimit(5)
            statusStr = self.status._getKeyValStr(["cmdMount", "iter"])
            self.writeToUsers("i", statusStr, cmd=self.currUserCmd)
            # convert from numpy to simple list for command formatting
            mount = [x for x in self.status.cmdMount]
            cmdMoveStr = self.formatGalilCommand(valueList=mount, cmd="XQ #MOVE")
            self.startDevCmd(cmdMoveStr, callFunc=self._moveIter, errFunc=self.failStop)
            return

        # done
        self._moveEnd()
    
    def _moveEnd(self, *args):
        """The final move device command ended successfully; clean up and set self.currUserCmd state done.

        Optionally perform any post-move cleanup (such as moving piezos).
        Finally: set self.currUserCmd state done (or failed, if cleanup failed).
        
        note: *args for callbackability...
        """
        self.currUserCmd.setState(self.currUserCmd.Done)

    def _statusCallback(self):
        """Callback for status command.
        """
        if not ('commanded position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Desired actuator positions not received\"", cmd=self.currUserCmd)
        if not ('actual position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Actual actuator positions not received\"", cmd=self.currUserCmd)
        if not ('status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Status word not received\"", cmd=self.currUserCmd)
        if not ('axis homed' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Homed axis info not received\"", cmd=self.currUserCmd)

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
        self.writeToUsers("i", statusStr, cmd = self.currUserCmd)
        self.currUserCmd.setState(self.currUserCmd.Done)

    def _clearDevCmd(self):
        """Clean up device command and associated data
        """
        self.parsedKeyList = []
        self.status.iter = numpy.nan
        if not self.currDevCmd.isDone:
            # failsafe, cmd should always be done at this point.
            self.currDevCmd.setState(self.currDevCmd.Failed)
        self.status.homing = [0]*self.nAct
        self.status.maxDuration = numpy.nan
        self.status.duration.reset()

    def formatGalilCommand(self, valueList, cmd, axisPrefix="", valFmt="%0.f", nAxes=None):
        """Format a Galil command
        
        Values that are None are replaced with MAXINT
        If len(valueList) < number of actuators, the extra axes are also set to MAXINT
    
        Inputs:
        - valueList: a list of values
        - cmd: the command (e.g. "XQ #MOVE")
        - axisPrefix: a string prefixing each axis
        - valFmt: value format
        - nAxes: number of axes in command; if None use all axes
        """
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
        GalilDevice.__init__(self,
            mirror = mirror,
            host = host,
            port = port,
            callFunc = callFunc,
        )
        # this mirror has extra status entries
        self.status.piezoStatus = numpy.nan
        self.status.piezoCorr = [numpy.nan]*3 
        
    def actOnKey(self, key, data):
        """An overwritten version from the base class to incorporate 2.5m Specific Keywords.
        Takes a key parsed from self.parseLine, and chooses what to do with the data
        
        inputs:
        -key: parsed descriptive string from self.parseLine
        -data: parsed data from self.parseLine, may be a numpy array
        """
        # look for piezo-specific parameters
        
#         You get this along with an XQ#SHOWPAR:
#         Check and try to parse these first as pPar Keys which will only be 
#         present in the piezo-specific actorkeys keyword dictionary.
#         
#          01.00 version of M2-specific additions
#         -00001676.4874,  00001676.4874 min, max piezo position (microsteps)
#          00002705 number of steps of piezo position
#          00000001.2400 resolution (microsteps/piezo ctrl bit)


        if key == 'version of M2-specific additions':
            # this is a special piezo parameter case, and can't be treated using sendParam
            msgStr = '%s=%s' % (('pParSoftwareVersion', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return
        elif key == 'min':
            msgStr = '%s=%s' % (('pParMinPos', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return
        elif key == 'max piezo position (microsteps)':
            msgStr = '%s=%s' % (('pParMaxPos', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return
        elif key == 'number of steps of piezo position':
            msgStr = '%s=%s' % (('pParNSteps', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return
        elif key == 'resolution (microsteps/piezo ctrl bit)':
            msgStr = '%s=%s' % (('pParResolution', data[0])) #data is a single element list
            self.writeToUsers("i", msgStr, cmd = self.currUserCmd)
            return
        # look for piezo-specific status
        elif key == 'piezo corrections (microsteps)':
            self.status.piezoCorr = [float(num) for num in data]
            updateStr = self.status._getKeyValStr(["piezoCorr"])
            outStr = self.formatAsKeyValStr("piezoStatusWord", data)
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return
        elif key == 'piezo status word':
            self.status.piezoStatus = int(data[0])
            updateStr = self.status._getKeyValStr(["piezoStatus"])
            self.writeToUsers("i", updateStr, cmd=self.currUserCmd)
            return
        else:
            # not piezo-related, try classical approach
            GalilDevice.actOnKey(self, key, data)

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

        Inputs:
        - cmd: passed automatically due to twistedActor callback framework
        """
        self.movePiezos()

    def _piezoMoveCallback(self, devCmd=None):
        """Called when the piezos are finished moving
        """
        if not self.currDevCmd.isDone:
            return

        if not('commanded position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Commanded actuator positions not received from piezo move\"", cmd=self.currUserCmd)
        if not('actual position' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Actual actuator positions not received from piezo move\"", cmd=self.currUserCmd)
        if not('status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Status word not received from piezo move\"", cmd=self.currUserCmd)
        if not('piezo status word' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Piezo status word not received from piezo move\"", cmd=self.currUserCmd)
        if not('piezo corrections (microsteps)' in self.parsedKeyList):
            self.writeToUsers("w", "Text=\"Piezo corrections not received from piezo move\"", cmd=self.currUserCmd)
        if self.currDevCmd.didFail:
            self.currUserCmd.setState(self.currUserCmd.Failed, "Piezo correction failed: %s" % (self.currDevCmd._textMsg,))
        else:
            self.currUserCmd.setState(self.currUserCmd.Done)
