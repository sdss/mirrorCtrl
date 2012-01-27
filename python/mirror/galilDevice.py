"""Galil Device.

This talks to a Galil motor controller, which commands actuators to move the mirror and 
reads from encoders to determine mirror position.

remember:
self.currDevCmd.cmdVerb.lower()


    The auxiliary port process will halt (in mid-output) if an ST or RS
    command is issued or the device is reset or power cycled. In the
    case of ST the aux output will start up (from the beginning of a new
    line) when the next XQ# command is issued. In the case of power
    cycle/reset, startup happens when the Galil finished resetting.
    Because of this, please sanity check the data carefully. 
    
    Use the "number of characters in a line" parameter! ???
    
    However, I don't
    recommend reading the data as fixed-width input unless you really
    think this adds safety, because the field widths may change. 
    
home: need galil response for parsing.
status: return isOK like filter wheel?, if motors are running, etc?

keywords not in status: reply, cmdFailed

when is a reply attributed to a devCmd vs userCmd?

format reply to user output
"""
import time
import itertools
import collections
import math
import re
import Tkinter

import numpy
import TclActor

ControllerAddr = 'localhost'
ControllerPort = 8000 # must match in twistedGalil.py for testing

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec
# scale orient array by this to convert into user-units
ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                                        MMPerMicron, MMPerMicron, RadPerArcSec], dtype = float)

# RegEx stuff up here to keep these from being needlessly re-compiled...
MatchNumBeg = re.compile('^.[0-9]*') # dot to allow negative sign
MatchNameEnd = re.compile('[a-z]*$')
MatchQMBeg = re.compile('^\?')

MaxMountErr = 1 # don't repeat move if mount difference is less (same for each axes, could use array)
MaxMoveIter = 2

class GalilTimer(object):
    """Keep track of execution time
    """
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.initTime = None
    
    def startTimer(self):
        """start time"""
        self.initTime = time.time()
        
    def getTime(self):
        """return time spent executing, so far
        """
        if not self.initTime:
            raise RuntimeError("No initial time present")
        else:
            return time.time() - self.initTime

class GalilStatus(object):
    """A container for holding the status of the Galil """
    def __init__(self, actor):
        """Initialize all keywords in StatusKeys """
        self.actor = actor
        self.mirror = actor.mirror
        self.nAct = len(self.mirror.actuatorList)
        # all the Status keyword/value pairs we care about, and their initial values
        # perhapps this shouldn't be an attribute...I'll leave it for now
        self.statusKeys = (
            ("userCmd", "?"),
            ("totalTime", numpy.nan),
            ("currExecTime", GalilTimer()), 
            ("remainExecTime", GalilTimer()),
            ("currActPos", [numpy.nan for x in range(self.nAct)]),
            ("desActPos", [numpy.nan for x in range(self.nAct)]), # updated upon iteration 
            ("currOrient", [numpy.nan for x in range(6)]),
            ("desOrient", [numpy.nan for x in range(6)]), 
            ("moveIter", numpy.nan), 
            ("maxMoveIter", MaxMoveIter), # adjusted below for non-encoder mirrors
            ("homing", ["?" for x in range(self.nAct)]),            
            ("axesHomed", ["?" for x in range(self.nAct)]), 
            ("statusWord", numpy.nan), 
            )
        # set each statusKey as a slot and initialize it.
        for keyword, init in self.statusKeys:
                if (not self.mirror.hasEncoders) and keyword=="maxMoveIter":
                    # mirrors without encoders shouldn't iterate on moves
                    setattr(self, keyword, 1)
                else:
                    setattr(self, keyword, init)    

    def _getKeyValStr(self, subset=None):
        """Package and return current keyword value info
        
        input:
        - subset: a list of keywords for which you wish to return a formatted string.
                    If unspecified (default) all keywords are returned.
        
        output:
        - statusStr: string in the correct keword value format to be sent to users
        """
        
        keywords = [key for key, initVal in self.statusKeys]
        if subset and (type(subset) ==  list):
            #find the indices of specific keywords in subset
            indices = [keywords.index(key) for key in subset]
            keywords = [keywords[index] for index in indices]
        if subset and (type(subset) == str):
            # find the index of specific keyword specifiec by subset
            index = keywords.index(subset)
            keywords = [keywords[index]] # outer brackets for an iterable object
         
        statusStr = '' 
        for keyword in keywords:
            val = getattr(self, keyword)
            if keyword == 'userCmd':
                # get the verb
                val = val.cmdVerb.lower()
            if keyword == 'currExecTime':
                # get current execution time
                val = val.getTime()
            if keyword == 'remainExecTime':
                # get current execution time
                    val = self.totalTime - self.currExecTime.getTime()
            if ('Orient' in keyword):
                # convert to user-friendly units
                val = numpy.divide(val, ConvertOrient)
            if type(val) in [list, numpy.ndarray]:
                # val is a list or numpy array, we need to format as comma seperated string
                strVal = ','.join(['%s' % (x) for x in val])
            else:
                strVal = '%s' % (val)
            statusStr += keyword + '=' + strVal + ';'
        # remove last trailing ';'
        statusStr = statusStr[:-1]
        return statusStr
                    

class GalilDevice(TclActor.TCPDevice):
    """The Galil Device Object
    """
    def __init__(self, callFunc = None, actor = None):
        TclActor.TCPDevice.__init__(self,
            name = "Galil",
            addr = ControllerAddr,
            port = ControllerPort,
            callFunc = callFunc,
            actor = actor,
            cmdInfo = (),  # like agile FW device?
        )
        self.mirror = actor.mirror
        self.currDevCmd = None # only 1 command can execute at a time
        self.replyList = None
        self.nAct = len(self.mirror.actuatorList)
        self.validAxisList =  ('A', 'B', 'C', 'D', 'E', 'F')[0:self.nAct]
        self.status = GalilStatus(actor)
        
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
        if not self.currDevCmd:
            # ignore unsolicited input
            return
        replyStr = replyStr.encode("ASCII", "ignore").strip(':\r\n')
        if not replyStr:
            # ignore blank replies
            return
        self.replyList.append(replyStr)
        if MatchQMBeg.match(replyStr.lstrip()): # if line begins with '?'
            # there was an error. End current process
            self.actor.writeToUsers("f", "cmdFailed; reply=%s" % (replyStr,), cmd = self.currDevCmd)
            self.currDevCmd.setState("failed", textMsg=replyStr)
            self.status.userCmd.setState("failed", textMsg=replyStr)
            self._clrDevCmd()        
            return
        if (self.status.userCmd.cmdVerb == 'move') and (len(self.replyList) == 1):
            #this line has move time estimates, put them in status
            self._setMoveTimeEst(replyStr)
            #self.actor.writeToUsers("i", "reply=Move time estimates from Galil: %s" % (replyStr,), cmd = self.currDevCmd)
            # add a time parser that updates status
        if not replyStr.lower().endswith("ok"):
            # command not finished
            return
        self.currDevCmd.setState("done")
            
    def newCmd(self, cmdStr, callFunc=None, userCmd=None):
        """Start a new device command.
        
        Slightly changed from base class definition.
        """
        # cmdStr is pre-formatted for Galil, might be the wrong way to use cmdClass... 
        cmd = self.cmdClass(cmdStr, userCmd=userCmd, callFunc=callFunc)
        if self.currDevCmd != None:
            raise RuntimeError("Cannot start new command, cmd slot not empty!")
        self.replyList = []
        self.currDevCmd = cmd
        self.status.currExecTime.startTimer()
        try:
            self.conn.writeLine(cmdStr)  #from fullCmdStr
        except Exception, e:
            cmd.setState("failed", textMsg=str(e))
            self._clrDevCmd()
            
    def cmdMove(self, orient, userCmd):
        """Accepts an orientation then commands the move.
        
        Subsequent moves are commanded until an acceptable orientation is reached (within errors).
        userCmd not tied to state of devCmd, because of subsequent moves. 
        """
        # if Galil is busy, abort the command
        if self.currDevCmd:
            #self.actor.writeToUsers("f", "cmdFailed; reply=Galil is busy", cmd = userCmd)
            userCmd.setState("cancelled", textMsg="Galil is busy")
            return
        
        userCmd.setState("running")
        self.status.userCmd = userCmd
        # enable iteration for mirrors with encoders
        if self.mirror.hasEncoders:
            self.status.moveIter = 1 #start counting from 1 or 0?
        # (user) commanded orient --> mount
        mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient=True)
        self.status.desActPos = numpy.asarray(mount, dtype=float) # the goal
        self.status.desOrient = numpy.asarray(adjOrient, dtype=float) # initial guess for fitter
        self.status.cmdActPos = numpy.asarray(mount, dtype=float) # this will change upon iteration.
        # format Galil command
        cmdMoveStr = self._galCmdFromMount(mount)
        updateStr = self.status._getKeyValStr("desOrient")
        self.actor.writeToUsers(">", updateStr, cmd=userCmd)
        self.newCmd(cmdMoveStr, userCmd=None, callFunc=self._moveCallback)
    
    def cmdStop(self, userCmd):
        """Sends an 'ST' to the Galil, causing it to stop whatever it is doing,
        waits 1 sec for motors to decelerate, then queries for status.
        """
        self.currDevCmd.setState("cancelled", textMsg="stop interrupt")
        self.status.userCmd.setState("cancelled", textMsg="stop interrupt")
        self._clrDevCmd() # stop listening for replies        
        userCmd.setState("running")
        self.conn.writeLine('ST;')
        time.sleep(1) # wait 1 second for deceleration
        self.cmdStatus(userCmd)
                
    def cmdStatus(self, userCmd):
        """Return the Galil status to the user.
        
        Called directly from the user, or indirectly through a cmdStop. userCmd 
        (cmd_stop or cmd_status) state tied to devCmd state.
        """
        if self.currDevCmd:
            self.actor.writeToUsers(">", "reply=Galil is busy, current status is...", cmd = self.currDevCmd)            
            statusStr = self.status._getKeyValStr()
            self.actor.writeToUsers("i", statusStr, cmd=userCmd)
            userCmd.setState("done")
            return
        userCmd.setState("running")
        self.status.userCmd = userCmd 
        self.actor.writeToUsers(">", "reply=Signaling Galil for Status", cmd=userCmd)
        self.newCmd("XQ #STATUS;", userCmd=None, callFunc = self._statusCallback)
                        
    def cmdParams(self, userCmd):
        """Show Galil parameters
        """
        if self.currDevCmd:
            #self.actor.writeToUsers("f", "cmdFailed; reply=Galil is busy", cmd = userCmd)
            userCmd.setState("cancelled", textMsg="Galil is busy")
            return    
        userCmd.setState("running")
        self.status.userCmd = userCmd  
        self.actor.writeToUsers(">", "reply=Signaling Galil", cmd=userCmd)
        self.newCmd("XQ #SHOWPAR;", userCmd=None, callFunc = self._paramsCallback)
        
    def cmdHome(self, axisList, userCmd):
        """Home the specified actuators
        
        Inputs:
        - axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        """
        if self.currDevCmd:
         #   self.actor.writeToUsers("f", "cmdFailed; reply=Galil is busy", cmd = userCmd)
            userCmd.setState("cancelled", textMsg="Galil is busy")
            return
        userCmd.setState("running")
        self.status.userCmd = userCmd
        self.actor.writeToUsers(">", "reply=Homing actuators: %s" % (axisList,), cmd = userCmd)
        # format Galil command
        axisList, cmdMoveStr = self._galCmdForHome(axisList)
        self.newCmd(cmdMoveStr, userCmd=None, callFunc=self._homeCallback)
        # send homing axes to status.
        homeStatus = numpy.zeros(self.nAct)
        for axis in axisList:
            ind = self.validAxisList.index(axis) # which one is homing?
            homeStatus[ind] = 1  # set it to 1
        self.status.homing = homeStatus
        # when move is done, check orientation from encoders

    def _moveCallback(self, cmd):
        """This code is executed when a device move command changes state.  If the cmd is
        done, then it parses the galil reply. Issues a subsequent move if necessary.
        
        Inputs: 
        - cmd: passed automatically due to TclActor callback framework
        
        todo: call for status update after every iter?
        """
        
        if not cmd.state == "done":
            return
        self.actor.writeToUsers("i", "reply=Move Iteration %i Finished" % \
                                (self.status.moveIter), cmd = self.currDevCmd)
        encMount = self._mountFromGalTxt()
        orient = self.mirror.orientFromEncoderMount(encMount, self.status.desOrient)
        actMount = self.mirror.actuatorMountFromOrient(orient)
        actMount = numpy.asarray(actMount, dtype=float)
        # add these new values to status
        self.status.currOrient = numpy.asarray(orient[:])
        self.status.currActPos = actMount
        actErr = self.status.desActPos - actMount
        # do another iteration?
        if True in (numpy.abs(actErr) > MaxMountErr) and \
                    (self.status.moveIter < self.status.maxMoveIter):
                # not within acceptable error range and more iterations are available
                # do another move
                # add offset to previous command
                self.status.desActPos += actErr
                # clear or update the relevant slots before beginning a new device cmd
                self.replyList = []
                self.currDevCmd = None
                self.status.moveIter += 1
                self.status.currExecTime.reset() # new timer for new move
                cmdMoveStr = self._galCmdFromMount(self.status.desActPos)
                # note: self.currDevCmd will be overwritten in newCmd, but never set to None, so 
                # shouldn't encounter race condition with other commands.
                self.actor.writeToUsers("i", "moveIter=%s" % (self.status.moveIter), \
                                         cmd = self.status.userCmd)
                self.newCmd(cmdMoveStr, userCmd=None, callFunc=self._moveCallback)
                # send a new command and clear the reply list and start over
                # note: self.currDevCmd is unchanged
                return
        # no more iterations, free up the Galil...        
        self._clrDevCmd()
        self.status.userCmd.setState("done")

    def _statusCallback(self, cmd):
        """Parse a reply from Galil after a status query.  
        
        This is a callback function for a status command.
        
        Inputs:
        - cmd: passed automatically because this is a callback.
        
        Note: different mirrors may have different returns for status! This is coded
        so that the correct lines are always grabbed by checking Galil line descriptors,
        called galKeywords below.
        
        could engineer some better more readable regexes... look at _setMoveTimeEst
        """
        # because this is a command callback
        if not cmd.state == "done":
            return
        # sample line (before parse):' -001497400, -000767250, -001199000 commanded position'
        # 2D array of values. Keywords will be smashed together with numumber at
        # in last list entry of each line...
        statList = [line.replace(' ', '').split(',') for line in self.replyList]
        del statList[-1] # this should be the 'OK' line received from Galil (nothing useful).
        galKeywords = []
        for line in statList:
            # regExes compiled up top
            # search last entry of each line for a status txt
            galKeywords.append(MatchNameEnd.search(line[-1]).group(0))
            # keep only the number in the last entry
            line[-1] = MatchNumBeg.search(line[-1]).group(0)
        
        # homed axes?
        ind = galKeywords.index('axishomed')
        self.status.axesHomed = [int(stat) for stat in statList[ind]]
        
        # commanded position
        ind = galKeywords.index('commandedposition')
        self.status.cmdActPos = [int(stat) for stat in statList[ind]]
        
        # measured position (adjusted)
        # account for encoder --> actuator spatial difference
        # this code below is also in handleReply(), could be broken out.
        ind = galKeywords.index('actualposition')
        encMount = numpy.asarray(statList[ind], dtype=float)
        # start initial guess for fitter from zeros, to be safe
        # because self.desOrient may not be defined
        # initOrient = self.desOrient
        initOrient = numpy.zeros(6)
        orient = self.mirror.orientFromEncoderMount(encMount, initOrient)
        actMount = self.mirror.actuatorMountFromOrient(orient)
        actMount = numpy.asarray(actMount, dtype=float)
        # add these new values to status
        self.status.currOrient = numpy.asarray(orient[:]) 
        self.status.currActPos = actMount    
        
        # status word
        ind = galKeywords.index('statusword')
        # zeropad status word to 32 bits
        self.status.statusWord = [int(stat) for stat in statList[ind]]        
        
        statusStr = self.status._getKeyValStr()
        self.actor.writeToUsers("i", statusStr, cmd = self.currDevCmd)
        self._clrDevCmd()
        self.status.userCmd.setState("done")
        
    def _paramsCallback(self, cmd):
        """Parse and print the parameters returned from the XQ# SHOWPAR query to the Galil
        
        This is a callback function for a cmdParams command.
        
        Inputs:
        - cmd: passed automatically because this is a callback.
        """
        # because this is a command callback
        if not cmd.state == "done":
            return
        
        for line in self.replyList[0:-2]: # don't include 'OK'
            # could parse into individual keywords?
            self.actor.writeToUsers("i", "reply=%s" % (line.lstrip()), cmd = None)
        self._clrDevCmd()    
        self.status.userCmd.setState("done")
 
    def _homeCallback(self, cmd):
        """This code is executed when a device cmdHome command changes state.  If the cmd is
        done, then it parses the galil reply.
        
        Inputs: 
        - cmd: passed automatically due to TclActor callback framework
        """        
        if not cmd.state == "done":
            return           
        # parse info needed
        self._clrDevCmd() 
        self.status.userCmd.setState("done")
         
    def _clrDevCmd(self):
        """Clean up device command when finished.
        """
        self.replyList = None
        self.status.moveIter = numpy.nan
        if not self.currDevCmd.isDone():
            # failsafe, cmd should always be done.
            self.currDevCmd.setState("failed")
        self.currDevCmd = None
        self.status.homing = [0 for x in range(self.nAct)]
        self.status.currExecTime.reset()

    def _setMoveTimeEst(self, replyStr):
        """Parses the line (Gaili reply from 'cmdMove') with move time estimates 
        for each actuator and puts the longest time estimate in status.
                
        Inputs:
        - replyStr: the correct line from Gailil with move time estimates
        """
        numList = replyStr.replace(' ', '').split(',')
        # grab only digits (text is smashed at end of replyStr)
        numList = [float(re.search('\d+.\d+', num).group(0)) for num in numList]
        numList = numpy.asarray(numList, dtype=float)
        maxTime = numpy.max(numList) # get time for longest move
        self.status.totalTime = maxTime
        updateStr = self.status._getKeyValStr(["totalTime", "currExecTime", "remainExecTime"])
        self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
 
    def _galCmdFromMount(self, mountList):
        """Converts mount information into a string command for a Galil
        
        notes: 
        The actuator mechanical/positional data needs to be defined in the
        correct order for each mirror...no way to check if that was done once
        we're here.
        """
        if len(mountList) > self.nAct:
            raise RuntimeError("Too many mount values")
        argList = []
        for ind, mount in enumerate(mountList):
            # axes to move
            argList.append("%s=%.0f;" % (self.validAxisList[ind], mount))
        for ind in range(len(mountList), self.nAct):
            # axes to leave alone (paranoia)
            argList.append("%s=MAXINT;" % (self.validAxisList[ind],))
        return " ".join(argList) + 'XQ #MOVE'
        
    def _galCmdForHome(self, axisList):
        """Format Galil home command from a list of axes
        
        Input:
        - axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        
        Return:
        - final axis list (full set if [] supplied; forced to uppercase)
        - command string
        
        Raise TclActor.Command.CommandError if invalid axes are specified
        """
        if not axisList:
            axisList = self.validAxisList
        else:
            axisList = [axis.upper() for axis in axisList]

        axisSet = set()
        validAxisSet = set(self.validAxisList)
        
        invalidAxisSet = axisSet - validAxisSet
        if invalidAxisSet:
            invalidAxisList = sorted(list(invalidAxisSet))
            raise TclActor.Command.CommandError(
                "Invalid axes %s; must be in: %s" % (invalidAxisList, self.validAxisList))

        argList = []
        for axis in self.validAxisList:
            if axis in axisSet:
                argList.append("%s=1" % (axis,))
            else:
                argList.append("%s=MAXINT" % (axis,))
        return axisList, " ".join(argList) + 'XQ #HOME'
                  
    def _mountFromGalTxt(self):
        """Parse text returned from Galil into a mount
        
        Assumes self.replyList has successfully put each Galil reply  as seperate list item
        """
        # only use final position output (3rd) line
        mntList = self.replyList[2].split(',')
        if 'final position' not in mntList[-1]:
            self.actor.writeToUsers("f", "reply=Parsing Error", cmd = self.currDevCmd)
        # print 'mntList: ', mntList
        mntList = [int(mnt.strip(' finalposition')) for mnt in mntList]
        mount = numpy.asarray(mntList, dtype=float)
        # print 'Recovered Mount!: ', mount
        return mount  
                     

        
        
        
        