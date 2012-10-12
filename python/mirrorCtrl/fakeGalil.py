#!/usr/bin/env python
"""Fake 3.5m M3 Galil.

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
__all__ = ["FakeGalilProtocol", "FakeGalilFactory"]

import re
import sys
import numpy

from mirrors import mir35mTert
from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol
from RO.Comm.TwistedTimer import Timer

MAXINT = 2147483647

MaxCmdTime = 2.0 # maximum time any command can take; sec

class FakeGalilProtocol(Protocol):
    lineEndPattern = re.compile(r"(\r\n|;)")
    
    def __init__(self, factory):
        self.factory = factory
        self._buffer = ''
        self.replyTimer = Timer()
        self.nAxes = 3
        
        homed, notHomed = numpy.array([1]*6, dtype=int), numpy.array([0]*6, dtype=int)
        self.isHomed = homed if self.factory.wakeUpHomed else notHomed
        self.cmdPos = numpy.array([0]*6, dtype=int)
        self.measPos = numpy.array([0]*6, dtype=int)
        self.userNums = numpy.array([MAXINT]*6, dtype=int)

        self.range = numpy.array([3842048]*3 + [190000]*3, dtype=int)
        self.speed = numpy.array([50000]*3 + [5000]*3, dtype=int)
        self.homeSpeed = numpy.array([5000]*3 + [500]*3, dtype=int)
        self.accel =  numpy.array([500000]*3 + [50000]*3, dtype=int)
        self.minCorr = numpy.array([0]*6, dtype=int)
        self.maxCorr = numpy.array([1000000]*3 + [15000]*3, dtype=int)
        self.st_fs = numpy.array([50]*6, dtype=int)
        self.marg = numpy.array([400000]*3 + [5000]*3, dtype=int)
        self.indSep = numpy.array([0]*6, dtype=int)
        self.encRes =  numpy.array([-3.1496]*3 + [1.5750]*3, dtype=float)
        self.status =  numpy.array([8196*6]*3, dtype=int)

    def dataReceived(self, data):
        self._buffer += data
        self._checkLine()

    def _checkLine(self):
       res = self.lineEndPattern.split(self._buffer, maxsplit=1)
       if len(res) > 1:
            line, sep, self._buffer = res
            self.lineReceived(line, sep)
    
       if self._buffer:
           Timer(0.000001, self._checkLine) # or reactor.callLater

    def echo(self, line, delim):
        self.transport.write(line + delim + ':')

    def lineReceived(self, line, delim):
        """As soon as any data is received, look at it and write something back."""
        if self.factory.verbose:
            print "received: %r" % (line,)
        cmd = line.strip()
        self.processCmd(cmd, delim)
        
    def processCmd(self, cmd, delim):
        self.echo(cmd, delim)
        if cmd in ("ST", "RS"):
            if cmd == "RS":
                homed = numpy.array([1]*6, dtype=int)
                notHomed = numpy.array([0]*6, dtype=int)
                #self.isHomed[:] =  homed if self.factory.wakeUpHomed else notHomed
                self.isHomed = notHomed
                #self.cmdPos = numpy.array([999999999]*6, dtype=int)
                #self.measPos = numpy.array([999999999]*6, dtype=int)
                self.cmdPos = numpy.array([0]*6, dtype=int)
                self.measPos = numpy.array([0]*6, dtype=int)
            self.replyTimer.cancel()
            return

# Is there a use case for setting a variable to -MAXINT?
#        cmdMatch = re.match(r"([A-F]) *= *(-)?MAXINT$", cmd)
        cmdMatch = re.match(r"([A-F]) *= *MAXINT$", cmd)
        if cmdMatch:
            axisChar = cmdMatch.groups()[0]
            ind = ord(axisChar) - ord("A")
            self.userNums[ind] = MAXINT
            return
            
        cmdMatch = re.match(r"([A-F]) *= *((-)?\d+)$", cmd)
        if cmdMatch:
            axis = cmdMatch.groups()[0]
            val = int(cmdMatch.groups()[1])
            ind = ord(axis) - ord("A")
            self.userNums[ind] = val
            return
            
        cmdMatch = re.match(r"XQ *#([A-Z]+)", cmd)
        if not cmdMatch:
            self.sendLine("?")
            return
        
        if self.replyTimer.isActive:
            # Busy, so reject new command
            self.sendLine("?")
            return

        cmdVerb = cmdMatch.groups()[0]

        if cmdVerb == "MOVE":
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
        self.isHomed[:] = numpy.logical_and(self.isHomed[:], self.userNums == MAXINT)
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
            numpy.array(numpy.random.normal(0, self.maxCorr / 10.0, 6), dtype=int))

        for msgStr in [
            "041,  006.6 microsteps, sec to find full step",
            self.formatArr("%09d", posErr, "position error"),
        ]:
            self.sendLine(msgStr)
        self.showStatus()
        self.showParams()
        self.done()
    
    def resetUserNums(self):
        self.userNums[:] = MAXINT
    
    def formatArr(self, fmtStr, arr, suffix):
        return ", ".join([fmtStr % val for val in arr[0:self.nAxes]]) + " " + suffix
    
    def showStatus(self):
        notHomed = numpy.nonzero(self.isHomed==0)
        cmdPos = self.cmdPos[:]
        measPos = self.measPos[:]
        cmdPos[notHomed] = 999999999
        measPos[notHomed] = 999999999
        for msgStr in [
            self.formatArr(" %d", self.isHomed, "axis homed"),
            self.formatArr("%09d", cmdPos, "commanded position"),
            self.formatArr("%09d", measPos, "actual position"),
            self.formatArr("%09d", self.status, "status word"),
        ]:
            self.sendLine(msgStr)
    
    def showParams(self):
        for msgStr in [
            "02.10, %d software version, NAXES number of axes" % (self.nAxes,),
            "1, 0, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections",
            "00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME",
            self.formatArr("%09d", - self.range / 2, "-RNGx/2 reverse limits"),
            self.formatArr("%09d", self.range / 2, "RNGx/2 reverse limits"),
            self.formatArr("%09d", self.speed, "SPDx speed"),
            self.formatArr("%09d", self.homeSpeed, "HMSPDx homing speed"),
            self.formatArr("%09d", self.accel, "ACCx acceleration"),
            self.formatArr("%09d", self.minCorr, "MINCORRx min correction"),
            self.formatArr("%09d", self.maxCorr, "MAXCORRx max correction"),
            self.formatArr("%09d", self.st_fs, "ST_FSx microsteps/full step"),
            self.formatArr("%09d", self.speed, "speed"),
            self.formatArr("%09d", self.marg, "MARGx dist betw hard & soft rev lim"),
            self.formatArr("%09d", self.indSep, "INDSEP index encoder pulse separation"),
            self.formatArr("%09.4f", self.encRes, "ENCRESx encoder resolution (microsteps/tick)"),
        ]:
            self.sendLine(msgStr)
    
    def moveStart(self, newCmdPos):
        """Start moving to the specified newCmdPos
        """
        deltaPos = newCmdPos - self.cmdPos
        deltaTimeArr = deltaPos / numpy.array(self.speed, dtype=float)
        moveTime = min(deltaTimeArr.max(), MaxCmdTime)
        self.cmdPos = newCmdPos
        # find what encoder positions this corresponds to for this mirror
        # only use first 3 or an error is thrown, Three actuators, so three mount positions.
        noisyPos = newCmdPos + numpy.array(numpy.random.normal(0, 100, 6), dtype=int)
        measOrient = mir35mTert.Mirror.orientFromActuatorMount(noisyPos[0:3])
        measMount = numpy.hstack((mir35mTert.Mirror.encoderMountFromOrient(measOrient), noisyPos[3:])) #append last 3 'unused' axes
        self.measPos = measMount

        self.sendLine(self.formatArr("%4.1f", deltaTimeArr, "max sec for move"))
        self.sendLine(self.formatArr("%09d", self.cmdPos, "target position"))
        self.replyTimer.start(moveTime, self.moveDone)
    
    def moveDone(self):
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
        if self.factory.verbose:
            print "sending: %r" % (line,)
        self.transport.write(line + "\r\n")


class FakeGalilFactory(Factory):
    """Fake Galil protocol factory
    
    Example of use:
        
    from twisted.internet import reactor
    reactor.listenTCP(port, FakeGalilFactory(verbose=False))
    reactor.run()
    """
    def buildProtocol(self, addr):
        """Build a FakeGalilProtocol
        
        This override is required because FakeGalilProtocol needs the factory in __init__,
        but default buildProtocol only sets factory after the protocol is constructed
        """
        return FakeGalilProtocol(factory = self)
        
    def __init__(self, verbose=True, wakeUpHomed=True):
        """Create a FakeGalilFactory
        
        Inputs:
        - verbose: print input and output?
        """
        self.verbose = bool(verbose)
        self.wakeUpHomed = bool(wakeUpHomed)
