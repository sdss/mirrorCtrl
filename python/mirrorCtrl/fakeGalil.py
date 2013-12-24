#!/usr/bin/env python
"""Fake Galil.

Notes:
- If a Galil is busy running an XQ# command then:
  - If it receives a new one it outputs "?".
  - If it receives other commands (such as ST or A=value) it executes them as usual;
  Warning: ST and RS may stop output in mid-stream.
- Unlike a real Galil, this one receives entire lines at a time
  (though perhaps tweaking some network setting could fix that)
    
To do:
- Add proper support for different mirrors:
  - Set nAxes appropriate (3-6)
  - Include correct mirror-specific info in status and other command output

- Figure out how to set status more realistically.
"""
import collections
import re

import numpy
from RO.Comm.TwistedTimer import Timer
from RO.Comm.TwistedSocket import TCPServer

__all__ = ["FakeGalil", "FakePiezoGalil"]

MAXINT = 2147483647
MaxCmdTime = 2.0 # maximum time any command can take; sec

class FakeGalil(TCPServer):    
    def __init__(self, mirror, port=0, verbose=False, wakeUpHomed=True, stateCallback=None):
        """Fake Galil TCP server

        @param[in] mirror: a mirrorCtrl.mirror.Mirror object, the mirror this fake galil should emulate
        @param[in] port: port on which to listen for connections
        @param[in] verbose: bool. Print extra info to terminal?
        @param[in] wakeUpHomed: bool. Should actuators be homed upon construction or not
        @param[in] stateCallback: a function to call when this server changes state; receives one argument: this server
        """
        self.verbose = bool(verbose)
        self.wakeUpHomed = bool(wakeUpHomed)
        self.mirror = mirror
        self._cmdBuffer = collections.deque()
        self.replyTimer = Timer()
        self.nAxes = len(self.mirror.actuatorList)
        self.userSocket = None

        TCPServer.__init__(self,
            port=port,
            stateCallback=stateCallback,
            sockStateCallback=self.sockStateCallback,
            sockReadCallback=self.lineReceived,
            name="fake %s" % (mirror.name,),
        )

        self.isHomed = self.arr(wakeUpHomed, dtype=bool)
        self.cmdPos = self.arr(0)
        self.measPos = self.arr(0)
        self.userNums = self.arr(MAXINT)

        self.range = self.arrAB(3842048, 190000)
        self.speed = self.arrAB(50000, 5000)
        self.homeSpeed = self.arrAB(5000, 500)
        self.accel =  self.arrAB(500000, 50000)
        self.minCorr = self.arr(0)
        self.maxCorr = self.arrAB(1000000, 15000)
        self.st_fs = self.arr(50)
        self.marg = self.arrAB(400000, 5000)
        self.indSep = self.arr(0)
        self.encRes =  self.arrAB(-3.1496, 1.5750, dtype=float)
        self.status =  self.arr(8196*6)
        self.noiseRange = 700 # steps, +/- range for adding steps to a measurement
    
    def sockStateCallback(self, sock):
        if sock.isReady:
#             print "Set user=", sock
            self.userSock = sock
        else:
#             print "Delete user"
            self.userSock = None

    def arr(self, val, dtype=int):
        """Make an array of [val0,...] of length self.nAxes
        """
        return numpy.array([val]*self.nAxes, dtype=dtype)
        
    def arrAB(self, val0, val1, dtype=int):
        """Make an array of length self.nAxes as a truncation of [val0, val0, val0, val1, val1, val1]
        """
        return numpy.array([val0]*3 + [val1]*3, dtype=dtype)[0:self.nAxes]
    
    def lineReceived(self, sock):
        """Called each time data is received.
        
        This does not actually start a command for two reasons:
        - a single line may contain multiple commands (separated by semicolons)
        - commands can queue up

        @param[in] sock: socket containing a line of data
        """
        line = sock.readLine()
        if line:
            self._cmdBuffer.extend(line.split(";"))
            self._startNextCmd()

    def _startNextCmd(self):
        """Split line on linebreakes, and forward each peice to 
        self.linedReceived on at a time.
        """
        if self._cmdBuffer:
            cmdStr = self._cmdBuffer.popleft()
            self.newCmd(cmdStr)

            if self._cmdBuffer:
                Timer(0.000001, self._startNextCmd)

    def echo(self, line):
        """Send line + "\n:" back, emulating an echo from a galil

        @param[in] line: line received to be written back
        """
        if self.userSock:
            self.userSock.writeLine(line)

    def newCmd(self, cmdStr):
        """New command received

        @param[in] cmdStr: command
        """
        if self.verbose:
            print "received: %r" % (cmdStr,)
        cmdStr = cmdStr.strip()
        self.echo(cmdStr)
        if cmdStr in ("ST", "RS"):
            if cmdStr == "RS":
                self.isHomed = self.arr(False, dtype=bool)
                self.cmdPos = self.arr(0)
                self.measPos = self.arr(0)
                self.userNums = self.arr(MAXINT)
            self.replyTimer.cancel()
            return        
        self.processCmd(cmdStr)
        
    def processCmd(self, cmdStr):
        """Figure out what was commanded, and emulate galil behavior accordingly

        @param[in] cmdStr: string, a galil command
        """
# Is there a use case for setting a variable to -MAXINT?
#        cmdMatch = re.match(r"([A-F]) *= *(-)?MAXINT$", cmdStr)
        cmdMatch = re.match(r"([A-F]) *= *MAXINT$", cmdStr)
        if cmdMatch:
            axisChar = cmdMatch.groups()[0]
            ind = ord(axisChar) - ord("A")
            self.userNums[ind] = MAXINT
            return
            
        cmdMatch = re.match(r"([A-F]) *= *((-)?\d+)$", cmdStr)
        if cmdMatch:
            axis = cmdMatch.groups()[0]
            val = int(cmdMatch.groups()[1])
            ind = ord(axis) - ord("A")
            self.userNums[ind] = val
            return
            
        cmdMatch = re.match(r"XQ *#([A-Z]+)", cmdStr)
        if not cmdMatch:
            self.sendLine("?")
            return
        
        if self.replyTimer.isActive:
            # Busy, so reject new command
            self.sendLine("?")
            return

        cmdVerb = cmdMatch.groups()[0]

        if cmdVerb == "MOVE":
            # round user nums to nearest st_fs step
            #self.userNums = numpy.around(self.userNums/50.)*50.
            newCmdPos = numpy.where(self.userNums == MAXINT, self.cmdPos, self.userNums)
            self.moveStart(newCmdPos)

        elif cmdVerb == "MOVEREL":
            deltaPos = numpy.where(self.userNums == MAXINT, 0, self.userNums)
            newCmdPos = self.cmdPos + deltaPos
            self.move(newCmdPos)
        
        elif cmdVerb == "STATUS":
            self.showStatus()
            self.done()
        
        elif cmdVerb == "SHOWPAR":
            self.showParams()
            self.done()
        
        elif cmdVerb == "HOME":
            self.homeStart()
            
        elif cmdVerb == "STOP":
            self.done()

        else:
            self.sendLine("?")
            self.done()
    
    def homeStart(self):
        """Start homing
        """
        self.isHomed[:] = numpy.logical_and(self.isHomed, self.userNums == MAXINT)
        cmdPos = numpy.where(self.userNums == MAXINT, self.cmdPos, self.cmdPos - self.range)
        deltaPos = numpy.where(self.userNums == MAXINT, 0.0, -self.range)
        deltaTimeArr = numpy.abs(deltaPos / numpy.array(self.speed, dtype=float))
        moveTime = min(deltaTimeArr.max(), MaxCmdTime)
        
        self.sendLine(self.formatArr("%6.1f", deltaTimeArr, "max sec to find reverse limit switch"))
        
        self.replyTimer.start(moveTime, self.homeFoundHome)
        
    def homeFoundHome(self):
        """Home, second step: found reverse limit, now move away
        """
        deltaPos = numpy.where(self.userNums == MAXINT, 0.0, self.marg)
        deltaTimeArr = deltaPos / numpy.array(self.homeSpeed, dtype=float)
        moveTime = min(deltaTimeArr.max(), MaxCmdTime)

        self.sendLine(self.formatArr("%6.1f", deltaTimeArr, "max sec to move away from home switch"))

        self.replyTimer.start(moveTime, self.homeMovedAway)

    def homeMovedAway(self):
        """Home third step: moved away from limit switch
        """
        self.sendLine("Finding next full step")
        self.replyTimer.start(0.1, self.homeDone)
        
    def homeDone(self):
        """Homing finished
        """
        self.isHomed[:] = numpy.logical_or(self.isHomed, self.userNums != MAXINT)
        posErr = numpy.where(self.userNums == MAXINT, 999999999,
            numpy.array(numpy.random.normal(0, self.maxCorr / 10.0, self.nAxes), dtype=int))

        for msgStr in [
            "041,  006.6 microsteps, sec to find full step",
            self.formatArr("%09d", posErr, "position error"),
        ]:
            self.sendLine(msgStr)
        self.showStatus()
        self.showParams()
        self.done()
    
    def resetUserNums(self):
        """Reset user nums"""
        self.userNums[:] = MAXINT
    
    def formatArr(self, fmtStr, arr, suffix):
        """ Return a comma separated string of values and a suffix"""
        return ", ".join([fmtStr % val for val in arr]) + " " + suffix
    
    def showStatus(self):
        """Show the status"""
        notHomedIndices = numpy.nonzero(numpy.logical_not(self.isHomed))
        cmdPos = self.cmdPos[:]
        measPos = self.measPos[:]
        cmdPos[notHomedIndices] = 999999999
        measPos[notHomedIndices] = 999999999
        for msgStr in [
            self.formatArr(" %d", self.isHomed, "axis homed"),
            self.formatArr("%09d", cmdPos, "commanded position"),
            self.formatArr("%09d", measPos, "actual position"),
            self.formatArr("%09d", self.status, "status word"),
        ]:
            self.sendLine(msgStr)
    
    def showParams(self):
        """Show parameters"""
        for msgStr in [
            "02.10, %d software version, NAXES number of axes" % (self.nAxes,),
            "1, 0, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections",
            "00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME",
            self.formatArr("%09d", - self.range / 2, "-RNGx/2 reverse limits"),
            self.formatArr("%09d", self.range / 2, "RNGx/2 forward limits"),
            self.formatArr("%09d", self.speed, "SPDx speed"),
            self.formatArr("%09d", self.homeSpeed, "HMSPDx homing speed"),
            self.formatArr("%09d", self.accel, "ACCx acceleration"),
            self.formatArr("%09d", self.minCorr, "MINCORRx min correction"),
            self.formatArr("%09d", self.maxCorr, "MAXCORRx max correction"),
            self.formatArr("%09d", self.st_fs, "ST_FSx microsteps/full step"),
            self.formatArr("%09d", self.marg, "MARGx dist betw hard & soft rev lim"),
            self.formatArr("%09d", self.indSep, "INDSEP index encoder pulse separation"),
            self.formatArr("%09.4f", self.encRes, "ENCRESx encoder resolution (microsteps/tick)"),
        ]:
            self.sendLine(msgStr)
    
    def moveStart(self, newCmdPos):
        """Start moving to the specified newCmdPos

        @param[in] newCmdPos: [int]*number of actuators.
        """
        # first check that all axes are homed
        # for now, assume all axes need to be homed for a move
        # to do: allow any homed axis to move.
        deltaPos = numpy.abs(newCmdPos - self.cmdPos)
        moveInd = numpy.nonzero(deltaPos) # True elements to be moved
        toMove = self.isHomed[moveInd] # must be full of ones, otherwise fail the cmdStr
        if 0 in toMove:
            unhomed = numpy.asarray(numpy.abs(self.isHomed-1), dtype=int)
            unhomed = [str(x) for x in unhomed]
            unhomed = ','.join(unhomed)
            self.sendLine('?HMERR: some axes to be moved have not been homed: %s' % unhomed)
            self.done()
            return
            
        #deltaPos = numpy.abs(newCmdPos - self.cmdPos)
        deltaTimeArr = deltaPos / numpy.array(self.speed, dtype=float)
        moveTime = min(deltaTimeArr.max(), MaxCmdTime)
        self.cmdPos = newCmdPos
        # get random sample between -self.noiseRange and +self.noiseRange
        #noise = numpy.random.random_sample(size=newCmdPos.shape)*2.*self.noiseRange - self.noiseRange
        noise = numpy.zeros(len(newCmdPos))
        if 0 in self.encRes:
            # no noise should be added to any axis with a 0 encoder resolution
            zeroit = numpy.nonzero(self.encRes==0)
            noise[zeroit] = 0.
        trueOrient = self.mirror.orientFromActuatorMount(newCmdPos)
  #      print "true Orientation", [x/y for x,y in itertools.izip(trueOrient, ConvertOrient)]
        trueEnc = self.mirror.encoderMountFromOrient(trueOrient)
        # add noise to encoder measurement
        noisyEnc = trueEnc + noise
        self.measPos = noisyEnc
 #       print "fake Galil Meas Pos", self.measPos
#         noisyPos = newCmdPos + noise
#         measOrient = self.mirror.orientFromActuatorMount(noisyPos[0:3])
#         #measMount = numpy.hstack((self.mirror.encoderMountFromOrient(measOrient), noisyPos[3:])) #append last 3 'unused' axes
#         measMount = self.mirror.encoderMountFromOrient(measOrient)
#         self.measPos = measMount

        self.sendLine(self.formatArr("%4.1f", deltaTimeArr, "max sec for move"))
        self.sendLine(self.formatArr("%09d", self.cmdPos, "target position"))
        self.replyTimer.start(moveTime, self.moveDone)
    
    def moveDone(self):
        """Called when a move is done"""
        self.sendLine(self.formatArr("%09d", self.measPos, "final position"))
        self.done()
    
    def done(self):
        """Call when an XQ command is finished
        
        Reset userNums (A-F) and print OK
        """
        self.replyTimer.cancel()
        self.resetUserNums()
        self.sendLine("OK")
    
    def sendLine(self, line):
        """write a line to the socket

        @param[in] line: string to be written
        """
        if self.verbose:
            print "sending: %r" % (line,)
        if self.userSock:
            self.userSock.writeLine(line)
        else:
            print "Warning: no socket to send this to:", line

        
class FakePiezoGalil(FakeGalil):
    def __init__(self, mirror, port, verbose=False, wakeUpHomed=True, stateCallback=None):
        """A fake Galil with mock piezo behavior like the 2.5m M2 mirror

        @param[in] mirror: a mirrorCtrl.mirror.Mirror object, the mirror this fake galil should emulate
        @param[in] port: port on which to listen for connections
        @param[in] verbose: bool. Print extra info to terminal?
        @param[in] wakeUpHomed: bool. Should actuators be homed upon construction or not
        @param[in] stateCallback: a function to call when this server changes state; receives one argument: this server
        """
        FakeGalil.__init__(self, mirror=mirror, port=port, verbose=verbose, wakeUpHomed=wakeUpHomed, stateCallback=stateCallback)
        self.cmdPiezoPos = numpy.array([0]*3, dtype=int)
        self.userPiezoNums = numpy.array([MAXINT]*3, dtype=int)
    
    def processCmd(self, cmdStr):
        """Overwritten from base class to handle piezo commands also

        @param[in] cmdStr: command string
        """
        # piezo specifics
        #print 'cmdStr: ', cmdStr
        cmdMatch = re.match(r"LDESPOS([A-F]) *= *((-)?\d+)$", cmdStr)
        if cmdMatch:
            axis = cmdMatch.groups()[0]
            val = int(cmdMatch.groups()[1])
            ind = ord(axis) - ord("A")
            self.userPiezoNums[ind] = val
            return
 
 
        cmdMatch = re.match(r"XQ *#(L[A-Z]+)", cmdStr)
        if cmdMatch and self.replyTimer.isActive:
            # Busy, so reject new command
            self.sendLine("?")
            return
        elif cmdMatch and cmdMatch.groups()[0] == "LMOVE":
            # round user nums to nearest st_fs step
            newCmdPos = numpy.where(self.userPiezoNums == MAXINT, self.cmdPiezoPos, self.userPiezoNums)
            self.movePiezo(newCmdPos)
        else:
            # normal stuff
            FakeGalil.processCmd(self, cmdStr)
    
    def resetUserNums(self):
        """Reset user numbers"""
        self.userNums[:] = MAXINT
        self.userPiezoNums[:] = MAXINT

    def movePiezo(self, piezoPos):
        """Do a piezo move

        @param[in] piezoPos: [length]*number of peizos, move em here
        """
        self.showStatus()
        self.sendLine("3 piezo status word")
        self.sendLine(self.formatArr("%4.1f", piezoPos, "piezo corrections (microsteps)"))
        #self.replyTimer.start(1, self.done())
        self.done()
