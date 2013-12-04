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
__all__ = ["FakeGalilFactory", "FakePiezoGalilFactory"]

# remove this when done testing
import itertools
import math
import numpy
MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.asarray([MMPerMicron, RadPerArcSec, RadPerArcSec, MMPerMicron, MMPerMicron])

##

import re
import sys
import numpy

from mirrors import mir35mTert, mir25mSec
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
        self.mirror = self.factory.mirror        
        self.nAxes = len(self.factory.mirror.actuatorList)
        
        
        homed, notHomed = numpy.array([1]*6, dtype=int)[0:self.nAxes], numpy.array([0]*6, dtype=int)[0:self.nAxes]
        self.isHomed = homed if self.factory.wakeUpHomed else notHomed
        self.cmdPos = numpy.array([0]*6, dtype=int)[0:self.nAxes]
        #self.cmdPos = numpy.array([-117150, -115750, -126700])#  999999999, 999999999], dtype=int) # present from russells tests
        self.measPos = numpy.array([0]*6, dtype=int)[0:self.nAxes]
        #self.measPos = numpy.array([129, 1480, -2132])#,  999999999, 999999999])
        self.userNums = numpy.array([MAXINT]*6, dtype=int)[0:self.nAxes]

        self.range = numpy.array([3842048]*3 + [190000]*3, dtype=int)[0:self.nAxes]
        self.speed = numpy.array([50000]*3 + [5000]*3, dtype=int)[0:self.nAxes]
        self.homeSpeed = numpy.array([5000]*3 + [500]*3, dtype=int)[0:self.nAxes]
        self.accel =  numpy.array([500000]*3 + [50000]*3, dtype=int)[0:self.nAxes]
        self.minCorr = numpy.array([0]*6, dtype=int)[0:self.nAxes]
        self.maxCorr = numpy.array([1000000]*3 + [15000]*3, dtype=int)[0:self.nAxes]
        self.st_fs = numpy.array([50]*6, dtype=int)[0:self.nAxes]
        self.marg = numpy.array([400000]*3 + [5000]*3, dtype=int)[0:self.nAxes]
        self.indSep = numpy.array([0]*6, dtype=int)[0:self.nAxes]
        self.encRes =  numpy.array([-3.1496]*3 + [1.5750]*3, dtype=float)[0:self.nAxes]
        self.status =  numpy.array([8196*6]*6, dtype=int)[0:self.nAxes]
        self.noiseRange = 700 # steps, +/- range for adding steps to a measurement

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
        self.echo(cmd, delim)
        if cmd in ("ST", "RS"):
            if cmd == "RS":
                homed = numpy.array([1]*6, dtype=int)[0:self.nAxes]
                notHomed = numpy.array([0]*6, dtype=int)[0:self.nAxes]
                #self.isHomed[:] =  homed if self.factory.wakeUpHomed else notHomed
                self.isHomed = notHomed
                self.cmdPos = numpy.array([0]*6, dtype=int)[0:self.nAxes]
                self.measPos = numpy.array([0]*6, dtype=int)[0:self.nAxes]                
                self.userNums = numpy.array([MAXINT]*6, dtype=int)[0:self.nAxes]
            self.replyTimer.cancel()
            return        
        self.processCmd(cmd, delim)
        
    def processCmd(self, cmd, delim):
    
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
        self.userNums[:] = MAXINT
    
    def formatArr(self, fmtStr, arr, suffix):
        return ", ".join([fmtStr % val for val in arr]) + " " + suffix
    
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
        """
        # first check that all axes are homed
        # for now, assume all axes need to be homed for a move
        # to do: allow any homed axis to move.
        deltaPos = numpy.abs(newCmdPos - self.cmdPos)
        moveInd = numpy.nonzero(deltaPos) # True elements to be moved
        toMove = self.isHomed[moveInd] # must be full of ones, otherwise fail the cmd
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
        self.proto = FakeGalilProtocol(factory = self)
        return self.proto
        
    def __init__(self, verbose=True, wakeUpHomed=True, mirror=mir35mTert.Mirror):
        """Create a FakeGalilFactory
        
        Inputs:
        - verbose: print input and output?
        """
        self.verbose = bool(verbose)
        self.wakeUpHomed = bool(wakeUpHomed)
        self.mirror = mirror

        
class FakePiezoGalilProtocol(FakeGalilProtocol):
    """A fake Galil with mock piezo behavior like the 2.5m M2 mirror
    """
    def __init__(self, factory):
        FakeGalilProtocol.__init__(self, factory)
        self.cmdPiezoPos = numpy.array([0]*3, dtype=int)
        self.userPiezoNums = numpy.array([MAXINT]*3, dtype=int)
    
    def processCmd(self, cmd, delim):
        """Overwritten from base class to handle piezo commands also
        """
        # piezo specifics
        print 'cmd: ', cmd
        cmdMatch = re.match(r"LDESPOS([A-F]) *= *((-)?\d+)$", cmd)
        if cmdMatch:
            axis = cmdMatch.groups()[0]
            val = int(cmdMatch.groups()[1])
            ind = ord(axis) - ord("A")
            self.userPiezoNums[ind] = val
            return
 
 
        cmdMatch = re.match(r"XQ *#(L[A-Z]+)", cmd)
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
            FakeGalilProtocol.processCmd(self, cmd, delim)
    
    def resetUserNums(self):
        self.userNums[:] = MAXINT
        self.userPiezoNums[:] = MAXINT

    def movePiezo(self, piezoPos):
        """Do a piezo move
        """
        self.showStatus()
        self.sendLine("3 piezo status word")
        self.sendLine(self.formatArr("%4.1f", piezoPos, "piezo corrections (microsteps)"))
        #self.replyTimer.start(1, self.done())
        self.done()
          
    
class FakePiezoGalilFactory(FakeGalilFactory):
    def __init__(self, verbose=True, wakeUpHomed=True, mirror=mir25mSec.Mirror):
        FakeGalilFactory.__init__(self, verbose, wakeUpHomed, mirror)
        
    def buildProtocol(self, addr):
        """Build a FakeGalilProtocol
        
        This override is required because FakeGalilProtocol needs the factory in __init__,
        but default buildProtocol only sets factory after the protocol is constructed
        """
        self.proto = FakePiezoGalilProtocol(factory = self)
        return self.proto
        

        
        
