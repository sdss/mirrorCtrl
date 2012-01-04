"""Galil Device.

This talks to a Galil motor controller, which commands actuators to move the mirror and 
reads from encoders to determine mirror position.

questions/to do: should we run xq #status before move or home command to clear all variables?
when should we set cmd state to "running", should we do it to device cmd or user cmd?

remember:
self.currCmd.cmdVerb.lower()


    The auxiliary port process will halt (in mid-output) if an ST or RS
    command is issued or the device is reset or power cycled. In the
    case of ST the aux output will start up (from the beginning of a new
    line) when the next XQ# command is issued. In the case of power
    cycle/reset, startup happens when the Galil finished resetting.
    Because of this, please sanity check the data carefully. Use the
    "number of characters in a line" parameter! However, I don't
    recommend reading the data as fixed-width input unless you really
    think this adds safety, because the field widths may change. 
"""
import time
import itertools
import collections
import math
import re
import Tkinter

import numpy
import TclActor

# punk (office linux box) for testing: '172.28.191.182'
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

# all the Status keyword/value pairs we care about 
StatusKeys = ("currCmd", "currExecTime", "lastActPos",
            "desActPos", "cmdActPos",  "iterNum", "currOrient",
            "desOrient", "motorsOn", "stopCode", "axesHomed", "homing", "cmdFailed",
           "onFullStep", "forwardLimitSwitchEngaged", "reverseLimitSwitchEngaged")

MaxIter = 2 # for repeated movments 
MaxMountErr = 1 # don't repeat move if mount difference is less (same for each axes, could use array)

class GalilTimer(object):
    """Keep track of execution times
    """
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.initTime = None
    
    def startTimer(self):
        """start time"""
        self.initTime = time.time()
        
    def getTime(self):
        """return time spent executing, so far"""
        if not self.initTime:
            # this may never happen since status is queried before being reset...nevertheless
            return 'Execution Finished'
        else:
            return time.time() - self.initTime


class GalilStatus(object):
    """A container for holding the status of the Galil """
    def __init__(self, actor):
        """Initialize all keywords in StatusKeys as attributes = None, except for the timer """
        self.actor = actor
        for keyword in StatusKeys:
            if keyword == 'currExecTime':
                setattr(self, keyword, GalilTimer())
            else:
                setattr(self, keyword, None)   
                
    def _printToUsers(self):
        """Output the current status to users 
        
        note: currently prints seperate line for each status slot, we could format it
        to print a single block...
        """
        for keyword in StatusKeys:
            numpy.set_printoptions(precision=0, suppress=True)
            val = getattr(self, keyword)
            if ('Orient' in keyword) and (val != None):
                # convert orientations into um and arcsec, and print to higher precision
                numpy.set_printoptions(precision=2, suppress=True)
                val = val / ConvertOrient
            if keyword == 'currExecTime':
                # get current time on timer
                val = val.getTime()
            self.actor.writeToUsers("i", "%s = %s" % (keyword, val), cmd = None)
        

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
        self.currCmd = None # only 1 command can execute at a time
        self.replyList = []
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
        if not self.currCmd:
            # ignore unsolicited input
            return
        replyStr = replyStr.encode("ASCII", "ignore").strip(':\r\n')
        if not replyStr:
            # ignore blank replies
            return
        # print 'Galil Reply: ', replyStr
        self.replyList.append(replyStr)
        if MatchQMBeg.match(replyStr.lstrip()): # if line begins with '?'
            # there was an error. 
            self.actor.writeToUsers("f", "Galil Error!=%s" % (replyStr,), cmd = self.currCmd)
            self.currCmd.setState("failed", textMsg=replyStr)
            # end current process
            self._cmdCleanUp() # put this as a callback?
            return
        if (self.status.currCmd == 'move') and (len(self.replyList) == 1):
            """print estimated time for move, the first Galil reply from an XQ#MOVE cmd..."""
            self.actor.writeToUsers("i", "Move Time estimates from Galil", cmd = self.currCmd)
            self.actor.writeToUsers("i", "%s" % (replyStr,), cmd = self.currCmd)
        if not replyStr.lower().endswith("ok"):
            # command not finished
            return
        elif (self.status.currCmd == 'move') and (self.status.iterNum < MaxIter):
            # do another move before terminating current command
            self.actor.writeToUsers("i", "Move Iteration %i Finished" % (self.status.iterNum), cmd = self.currCmd)
            self.status.iterNum += 1
            encMount = self._mountFromGalTxt()
            orient = self.mirror.orientFromEncoderMount(encMount, self.status.desOrient)
            actMount = self.mirror.actuatorMountFromOrient(orient)
            actMount = numpy.asarray(actMount, dtype=float)
            # add these new values to status
            self.status.currOrient = numpy.asarray(orient[:])
            self.status.lastActPos = actMount
            actErr = self.status.desActPos - actMount
            if True in (numpy.abs(actErr) > MaxMountErr):
                # not within acceptable error range
                # do another move
                # add offset to previous command
                self.status.cmdActPos += actErr 
                cmdMoveStr = self._galCmdFromMount(self.status.cmdActPos)
                # send a new command and clear the reply list and start over
                # note: self.currCmd is unchanged
                self.replyList = []
                self.conn.writeLine(cmdMoveStr) 
                return
        self.currCmd.setState("done")
            
    def newCmd(self, cmdStr, callFunc=None, userCmd=None):
        """Start a new command.
        
        Slightly changed from base class definition.
        """
        # cmdStr is pre-formatted for Galil, might be the wrong way to use cmdClass... 
        cmd = self.cmdClass(cmdStr, userCmd=userCmd, callFunc=callFunc)
        self.currCmd = cmd
        self.status.currExecTime.startTimer()
        self.status.currCmd = userCmd.cmdVerb # most recent command received by GalilDevice
        try:
            self.conn.writeLine(cmdStr)  #from fullCmdStr
        except Exception, e:
            cmd.setState("failed", textMsg=str(e))
            
    def moveTo(self, orient, userCmd):
        """Accepts an orientation then commands the move.
        
        Subsequent moves are commanded until an acceptable orientation is reached (within errors)
        """
        # if Galil is busy, abort the command
        if self.currCmd:
            self.actor.writeToUsers("f", "Galil is busy", cmd = self.currCmd)
            return
        # enable iteration for mirrors with encoders
        if self.mirror.hasEncoders:
            self.status.iterNum = 0
        # (user) commanded orient --> mount
        mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient=True)
        self.status.desActPos = numpy.asarray(mount, dtype=float) # the goal
        self.status.desOrient = numpy.asarray(adjOrient, dtype=float) # initial guess for fitter
        self.status.cmdActPos = numpy.asarray(mount, dtype=float) # this will change upon iteration.
        self.status.cmdOrient = orient
        # format Galil command
        cmdMoveStr = self._galCmdFromMount(mount)
        self.newCmd(cmdMoveStr, userCmd=userCmd, callFunc=self._cmdCleanUp)
    
    def stop(self):
        """Sends an 'ST' to the Galil, causing it to stop whatever it is doing.
        """
        self.conn.writeLine('ST;')
        if self.currCmd:
            self.actor.writeToUsers("f", "aborted", cmd = self.currCmd)
        
    def getStatus(self, userCmd):
        """Return the Galil status to the user.
        """
        if not self.currCmd:
            # query the Galil and update/send status
            self.newCmd("XQ #STATUS;", userCmd=userCmd, callFunc = self._statusFromGalTxt)
            self.actor.writeToUsers("i", "Checking Galil", cmd = self.currCmd)
            return
        else:
            self.actor.writeToUsers("i", "Galil is busy (cannot send new query), known status is...", cmd = self.currCmd)            
        self.status._printToUsers()
        
    def showParams(self, userCmd):
        """Show Galil parameters
        """
        if self.currCmd:
            self.actor.writeToUsers("f", "Galil is busy", cmd = self.currCmd)
            return
            
        self.newCmd("XQ #SHOWPAR;", userCmd=userCmd, callFunc = self._paramsFromGalTxt)
        
    def home(self, axisList, userCmd):
        """Home the specified actuators
        
        Inputs:
        - axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        """
        if self.currCmd:
            self.actor.writeToUsers("f", "Galil is busy", cmd = self.currCmd)
            return

        # format Galil command
        axisList, cmdMoveStr = self._galCmdForHome(axisList)

        self.actor.writeToUsers("d", "Homing actuators: %s" % (axisList,))

        self.newCmd(cmdMoveStr, userCmd=userCmd, callFunc=self._cmdCleanUp)
        # send homing axes to status.
        homeStatus = numpy.zeros(self.nAct)
        for axis in axisList:
            ind = self.validAxisList.index(axis) # which one is homing?
            homeStatus[ind] = 1  # set it to 1
        self.status.homing = homeStatus
        # when move is done, check orientation from encoders
    
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

    def _paramsFromGalTxt(self, cmd):
        """Parse and print the parameters returned from the XQ# SHOWPAR query to the Galil
        
        cmd must be passed to this method for callback-ability
        """
        for line in self.replyList[0:-2]: # don't include 'OK'
            self.actor.writeToUsers("i", "%s" % (line.lstrip()), cmd = None)
        self._cmdCleanUp()       

    def _statusFromGalTxt(self, cmd=None):
        """Parse a reply from Galil after a status query.
        
        Inputs:
        - none (the cmd argument is ignored; it is only present to allow use as a callback function).
        
        Note: different mirrors may have different returns for status! This is coded
        so that the correct lines are always grabbed by checking Galil line descriptors,
        called galKeywords below.
        """      
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
        self.status.lastActPos = actMount    
        
        # status word
        ind = galKeywords.index('statusword')
        self._parseStatusWord(statList[ind])        
        
        self.status._printToUsers()
        self._cmdCleanUp()

    def _parseStatusWord(self, statusWordList):
        """Parse a list of status words (one for each axis), and update Galil Status accordingly
        """
        self.status.motorsOn = []
        self.status.onFullStep = []
        self.status.forwardLimitSwitchEngaged = []
        self.status.reverseLimitSwitchEngaged = []
        self.status.stopCode = []
        for ind, word in enumerate(statusWordList):
            # just keep numeric part, get rid of the leading '0b'
            inBin = bin(int(word))[2:]
            # zero pad to 32 bits, although most are unused.
            inBin = inBin.zfill(32  - len(inBin))
            # stop code, bits 1-8
            self.status.stopCode.append(int(inBin[-8:], 2))
            self.status.motorsOn.append(int(inBin[-14]))
            self.status.onFullStep.append(int(inBin[-17]))
            self.status.forwardLimitSwitchEngaged.append(int(inBin[-12]))
            self.status.reverseLimitSwitchEngaged.append(int(inBin[-11]))
                  
    def _mountFromGalTxt(self):
        """Parse text returned from Galil into a mount
        
        Assumes self.replyList has successfully put each Galil reply  as seperate list item
        """
        # only use final position output (3rd) line
        mntList = self.replyList[2].split(',')
        if 'final position' not in mntList[-1]:
            self.actor.writeToUsers("f", "Parsing Error", cmd = self.currCmd)
        # print 'mntList: ', mntList
        mntList = [int(mnt.strip(' finalposition')) for mnt in mntList]
        mount = numpy.asarray(mntList, dtype=float)
        # print 'Recovered Mount!: ', mount
        return mount  
                     
    def _cmdCleanUp(self, cmd=None):
        """Clean up when a command finishes

        Inputs:
        - none (the cmd argument is ignored; it is only present to allow use as a callback function).
        """
        self.replyList = []
        self.status.iterNum = None
        self.status.iterMax = None
        self.currCmd = None
        self.status.homing = None
        self.status.currExecTime.reset()
