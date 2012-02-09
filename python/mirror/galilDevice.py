"""Galil Device.

This talks to a Galil motor controller, which commands actuators to move the mirror and 
reads from encoders to determine mirror position.

There is lots of info at:
http://www.apo.nmsu.edu/Telescopes/HardwareControllers/GalilMirrorControllers.html#InterfaceReplies

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

startsWithNumRegEx = re.compile(r'^[0-9]|^-[0-9]')
getDataRegEx = re.compile(r'(?<!/)[0-9-.]+')
paramRegEx = re.compile(r'^[A-Z]|^-[A-Z]')
timeEstRegEx = re.compile(r'^sec +to +move|^max +sec')

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
        self.currDevCmd = self.cmdClass('null') # initialize empty command in cmd slot
        self.currDevCmd.setState("done") # set it to a done state so subsequent cmds can execute
        self.replyList = None
        self.nAct = len(self.mirror.actuatorList)
        self.validAxisList =  ('A', 'B', 'C', 'D', 'E', 'F')[0:self.nAct]
        self.status = GalilStatus(actor)
        self.expectedReplies = [
            'max sec for move', 
            'target position',
            'final position',
            'axis homed',
            'commanded position',
            'actual position', 
            'status word',
            'max sec to find reverse limit switch', # homing 
            'reverse limit switch not depressed for axes:', # data follows this one
            'trying again',
            'max sec to find home switch',
            'sec to move away from home switch',
            'finding next full step',
            'microsteps', #share line
            'sec to find full step' # share line
            'position error',
            'software version', #share line
            'NAXES number of axes', # share line
            'DOAUX aux status?', # share line
            'MOFF motors off when idle?', # share line
            'NCORR # corrections', # share line
            'WTIME', # share line
            'ENCTIME', # share line
            'LSTIME', # share line
            '-RNGx/2 reverse limits',
            'RNGx/2 forward limits',
            'SPDx speed',
            'HMSPDx homing speed',
            'ACCx acceleration',
            'MINCORRx min correction',
            'MAXCORRx max correction',
            'ST_FSx microsteps/full step',
            'MARGx dist betwe hard & soft rev lim',
            'INDSEP index encoder pulse separation',
            'ENCRESx encoder resolution (microsteps/tick)',
            ]

    def parseLine(self, line):
        """Tries to parse a reply from the Galil and seperate into descriptor-value format.
        I consider any descriptive text following the data a descriptor, there can be multiple 
        descriptor and values on a single line.
        
        output:
        - dataMatched: numpy array containing any data
        - keys: text describing any data, may be a list.
        
        example lines:
            0000.2,  0362.7,  0000.2,  0000.0,  0000.0 max sec to find reverse limit switch
             0300.1,  0300.1,  0300.1,  0000.0,  0000.0 max sec to find home switch
             0008.2,  0008.2,  0008.2,  0000.0,  0000.0 sec to move away from home switch
            Finding next full step
             041,  006.6 microsteps, sec to find full step
            -000006732,  000014944,  000003741,  999999999,  999999999 position error
             1,  1,  1,  0,  0 axis homed       
        """      
        # Grab the data in each line:
        # Match only numbers (including decimal pt and negative sign) that are not preceeded by '/'
        # This is due to the param named 'RNGx/2' which I don't want to match
        
        dataMatched = getDataRegEx.findall(line) # will put non-overlapping data (numbers) in a list
        dataMatched = numpy.asarray(dataMatched, dtype=float)
        numVals = len(dataMatched)
        # Grab descriptor info on each line, which follows data.
        # split on whitespace n times, last split will be descriptor info
        textOnly = re.split(r' +', line, maxsplit = (numVals +1))[-1]
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
        missMatch = ((len(keys) > 1) and (len(keys) != len(dataMatched))) or\
                    ((len(keys) == 1) and (len(dataMatched) != self.nAct)) or\
                    ((len(keys) == 1) and (len(dataMatched) != 1))
        if missMatch:
            # There is confusion in the amount of descriptors and values in this particular line.
            # report it but keep going
            self.actor.writeToUsers("w", "Suspicious Galil Parse: %s, num descriptors do not match num values." % (line,), cmd=self.status.userCmd)
            return
        # check to see if descriptor/s are an expected reply
        knownReply = [(key in self.expectedReplies) for key in keys]
        if False in knownReply:
            # a descriptor wasn't recognized as an expected reply for this Galil/mirror
            # report it but keep going
            self.actor.writeToUsers("w","Suspicious Galil Parse: %s, Descriptor text not recognized." % (line,), cmd=self.status.userCmd)
            return
        return dataMatched, keys
        
    def sendGalilParam(self, key, data):
        """Key is determined to be a Galil parameter, send the key and data to the user.
        Keyword for Galil Paramaters are 'GalilPar<par>' where par is the name from the
        Galil itself.
        
        key example: -RNGx/2 reverse limits
       
        """
        param = key.split()[0] # in example: just keep -RNGx/2
        dataStr = ','.join(['%s' % (x) for x in data]) # comma separate
        str = 'GalilPar%s=%s' % ((param, dataStr))
        self.actor.writeToUsers("i", str, cmd = self.status.userCmd)        

    def actOnKey(self, key, data):
        """Takes a key parsed from self.parseLine, and chooses what to do with the data
        
        inputs:
        -key: parsed descriptive string from self.parseLine
        -data: parsed data from self.parseLine, may be a numpy array
        """
         #line begins with cap letter (or -cap letter)
        
        if paramRegEx.match(key):
            # this is a Galil parameter, format it and send it to the user
            self.sendGalilParam(key, data)
            return
            
        elif 'software version' in key:
            # this is a special parameter case, and can't be treated using sendParam
            str = 'GalilPar%s=%s' % ((param, data[0])) #data is a single element list
            self.actor.writeToUsers("i", str, cmd = self.status.userCmd)
            return
                   
        elif timeEstRegEx.match(key):
            # contains information about estimated execution time
            # update status and notify users
            maxTime = numpy.max(data) # get time for longest move
            self.status.totalTime = maxTime
            updateStr = self.status._getKeyValStr(["totalTime", "currExecTime", "remainExecTime"])
            self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
            # update cmd timeouts here!
            return
        
        elif 'axis homed' in key:
            self.status.axesHomed = [int(num) for num in data]
            updateStr = self.status._getKeyValStr(["axisHomed"])
            self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
            return
        
        elif ('commanded position' in key) or ('target position' in key):
            self.status.cmdActPos = [int(num) for num in data]
            updateStr = self.status._getKeyValStr(["cmdActPos"])
            self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
            return

        elif ('actual position' in key) or ('final position' in key):        
            # measured position (adjusted)
            # account for encoder --> actuator spatial difference
            encMount = data[:]
            # desOrient may be nans
            if numpy.isfinite(sum(self.desOrient)):
                initOrient = self.desOrient
            else:
                initOrient = numpy.zeros(6)
            orient = self.mirror.orientFromEncoderMount(encMount, initOrient)
            actMount = self.mirror.actuatorMountFromOrient(orient)
            actMount = numpy.asarray(actMount, dtype=float)
            # add these new values to status
            self.status.currOrient = numpy.asarray(orient[:]) 
            self.status.currActPos = actMount
            updateStr = self.status._getKeyValStr(["currOrient", "currActPos"])
            self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
            return
        
        elif 'status word' in key:
            # zeropad status word to 32 bits
            self.status.statusWord = [int(num) for num in data]
            updateStr = self.status._getKeyValStr(["statusWord"])
            self.actor.writeToUsers("i", updateStr, cmd=self.status.userCmd)
            return
        
        else:
            # send the remaining info back to the user
            str = "reply=%s: %s" % (key, data)
            self.actor.writeToUsers("i", str, cmd=self.status.userCmd)
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
        if self.currDevCmd.isDone():
            # ignore unsolicited input
            return
        replyStr = replyStr.encode("ASCII", "ignore").strip(' :\r\n')
        if not replyStr:
            # ignore blank replies
            return
        self.replyList.append(replyStr)  # add reply to buffer        
        if MatchQMBeg.match(replyStr): # if line begins with '?'
            # there was an error. End current process
            self.actor.writeToUsers("f", "reply=Galil Error: %s" % (replyStr,), cmd = self.currDevCmd)
            self.currDevCmd.setState("failed", textMsg=replyStr)
            self.status.userCmd.setState("failed", textMsg=replyStr)
            self._clrDevCmd()        
            return
        okLineRegEx = re.compile(r'^OK$', re.IGNORECASE)
        if okLineRegEx.match(replyStr):
            # command finished
            self.currDevCmd.setState("done")
            return       
                
        if not startsWithNumRegEx.match(replyStr):
            # line doesn't begin with a data value, eg 'Finding next full step'
            # show it to the user and return
            self.actor.writeToUsers("i", "reply=Galil Line: %s" % (replyStr,), cmd = self.status.userCmd)
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
            # if one key, all the data goes with it
            self.actOnKey(description, data)
        else:
            # one data value for each key
            for key, dat in itertools.izip(description, data):
                self.actOnKey(key, dat)

            
    def newCmd(self, cmdStr, callFunc=None, userCmd=None):
        """Start a new device command.
        
        Slightly changed from base class definition.
        """
        # cmdStr is pre-formatted for Galil, might be the wrong way to use cmdClass... 
        cmd = self.cmdClass(cmdStr, userCmd=userCmd, callFunc=callFunc)
        if not self.currDevCmd.isDone():
            raise RuntimeError("Cannot start new command, there is a cmd currently running")
        self.replyList = []
        self.currDevCmd = cmd
        self.status.currExecTime.startTimer()
        try:
            self.conn.writeLine(cmdStr)  #from fullCmdStr
        except Exception, e:
            cmd.setState("write to device failed", textMsg=str(e))
            self._clrDevCmd()
            
    def cmdMove(self, orient, userCmd):
        """Accepts an orientation then commands the move.
        
        Subsequent moves are commanded until an acceptable orientation is reached (within errors).
        userCmd not tied to state of devCmd, because of subsequent moves. 
        """
        # if Galil is busy, abort the command
        if not self.currDevCmd.isDone():
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
        if not self.currDevCmd.isDone():
            self.actor.writeToUsers(">", "reply=Galil is busy, current status is...", cmd = self.currDevCmd)            
            statusStr = self.status._getKeyValStr()
            self.actor.writeToUsers("i", statusStr, cmd=userCmd)
            userCmd.setState("done")
            return
        userCmd.setState("running")
        self.status.userCmd = userCmd 
        self.actor.writeToUsers(">", "reply=Signaling Galil for Status", cmd=userCmd)
        self.newCmd("XQ #STATUS;", userCmd=userCmd, callFunc = self._userCmdCallback)
                        
    def cmdParams(self, userCmd):
        """Show Galil parameters
        """
        if not self.currDevCmd.isDone():
            #self.actor.writeToUsers("f", "cmdFailed; reply=Galil is busy", cmd = userCmd)
            userCmd.setState("cancelled", textMsg="Galil is busy")
            return    
        userCmd.setState("running")
        self.status.userCmd = userCmd  
        self.actor.writeToUsers(">", "reply=Signaling Galil", cmd=userCmd)
        self.newCmd("XQ #SHOWPAR;", userCmd=userCmd, callFunc = self._userCmdCallback)
        
    def cmdHome(self, axisList, userCmd):
        """Home the specified actuators
        
        Inputs:
        - axisList: a list of axes to home (e.g. ["A", "B", C"]) or None or () for all axes; case is ignored
        """
        if not self.currDevCmd.isDone():
         #   self.actor.writeToUsers("f", "cmdFailed; reply=Galil is busy", cmd = userCmd)
            userCmd.setState("cancelled", textMsg="Galil is busy")
            return
        userCmd.setState("running")
        self.status.userCmd = userCmd
        self.actor.writeToUsers(">", "reply=Homing actuators: %s" % (axisList,), cmd = userCmd)
        # format Galil command
        axisList, cmdMoveStr = self._galCmdForHome(axisList)
        self.newCmd(cmdMoveStr, userCmd=userCmd, callFunc=self._userCmdCallback)
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
        actErr = self.status.desActPos - self.status.currActPos
        # do another iteration?
        if True in (numpy.abs(actErr) > MaxMountErr) and \
                    (self.status.moveIter < self.status.maxMoveIter):
                # not within acceptable error range and more iterations are available
                # do another move
                # add offset to previous command
                self.status.desActPos += actErr
                # clear or update the relevant slots before beginning a new device cmd
                self.replyList = []
                #self.currDevCmd.setState("done")
                self.status.moveIter += 1
                self.status.currExecTime.reset() # new timer for new move
                cmdMoveStr = self._galCmdFromMount(self.status.desActPos)
                # note: self.currDevCmd will be overwritten in newCmd, but never set to None, so 
                # shouldn't encounter race condition with other commands.
                self.actor.writeToUsers("i", "moveIter=%s" % (self.status.moveIter), \
                                         cmd = self.status.userCmd)
                self.newCmd(cmdMoveStr, userCmd=self.status.userCmd, callFunc=self._moveCallback)
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

    def _userCmdCallback(self, cmd):
        """Generic callback that sets the user cmd done
        
        Inputs:
        -cmd: passed automatically due to TclActor callback framework
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
            # failsafe, cmd should always be done at this point.
            self.currDevCmd.setState("failed")
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
                     

class GalilDevice35M3(GalilDevice):
    """A Galil Device controller that is specific to the 3.5m Tertiary mirror
    """
    def __init__(self, callFunc = None, actor = None):
        GalilDevice.__init__(self, callFunc = None, actor = None)
        self.expectedReplies.extend([
            'version of M3-specific additions',
            'off-on-error?',
            'error limit for tertiary rotation',
            'time to close rotation clamp', 
            'open clamp',
            'turn on at-slot sensor (sec)',
            'max time',
            'poll time',
            'addtl run time for primary mirror cover motion (sec)',
            'time for primary mirror eyelid motion (sec)',
            'sec to finish move'
            ])
        

class GalilDevice25M2(GalilDevice):
    """A Galil Device controller that is specific to the 2.5m Secondary mirror
    
    note: add in piezo corrections for move
    """
    def __init__(self, callFunc = None, actor = None):
        GalilDevice.__init__(self, callFunc = None, actor = None)
        self.expectedReplies.extend([
            'piezo status word', 
            'piezo corrections (microsteps)',
            'version of M2-specific additions',
            'min, max piezo position (microsteps)',
            'number of steps of piezo position',
            'resolution (microsteps/piezo ctrl bit)'
            ])
            