#!/usr/bin/env python
"""Fake 3.5m M3 Galil.

To do:
- Make command echo more realistic; see how real Galils do it.
- Handle command collisions (receiving a new command while another is running) correctly --
  study how a real Galil does it!
- Support XQ#STOP; what is the correct output for that command?
- Add proper support for different mirrors:
  - Set nAxes appropriate (3-6)
  - Include correct mirror-specific info in status and other command output
- Perhaps support waking up homed or not homed.
  That provides more realistic normal operation (it should wake up not homed)
  while allowing fast unit testing (wake up homed, as it does now).
- Figure out how to set status more realistically.
"""
DefaultPort = 8000

import re
import sys
import numpy
from twisted.internet import reactor, protocol, defer
from twisted.protocols.basic import LineReceiver
from RO.Comm.TwistedTimer import Timer

Debug = True

MAXINT = 2147483647

MaxCmdTime = 2.0 # maximum time any command can take; sec

class FakeGalilProtocol(LineReceiver):
    def __init__(self, *args, **kwargs):
        self.replyTimer = Timer()
        self.nAxes = 5

        self.isHomed = numpy.array([1]*6, dtype=int)
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
        self.status =  numpy.array([8196*6], dtype=int)
    
    def echo(self, line):
        self.sendLine(line + ":") # not quite right, but maybe close

    def lineReceived(self, line):
        """As soon as any data is received, look at it and write something back."""
        if Debug:
            print "received: %r" % (line,)
        cmdList = [c.strip() for c in line.split(";") if c.strip()]
        for cmd in cmdList:
            self.processCmd(cmd)
        
    def processCmd(self, cmd):
        if Debug:
            print "processCmd(cmd=%r)" % (cmd,)
        self.echo(cmd)
        if cmd in ("ST", "RS"):
            self.replyTimer.cancel()
            return
        
        cmdMatch = re.match(r"([A-F]) *= *MAXINT$", cmd)
        if cmdMatch:
            axisChar = cmdMatch.groups()[0]
            ind = ord(axisChar) - ord("A")
            self.userNums[ind] = MAXINT
            return
            
        cmdMatch = re.match(r"([A-F]) *= *(\d+)$", cmd)
        if cmdMatch:
            axis = cmdMatch.groups()[0]
            val = int(cmdMatch.groups()[1])
            ind = ord(axis) - ord("A")
            self.userNums[ind] = val
            return
            
        cmdMatch = re.match(r"XQ *#([A-Z]+)", cmd)
        if not cmdMatch:
            return
        
        if self.replyTimer.isActive:
            # Reject new command; but what does the Galil really do?
            # It probably accepts non-XQ commands such as A=value, but what about XQ commands?
            self.sendLine("? Busy")
            self.sendLine("OK")
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

        else:
            self.sendLine("? Unrecognized command %s" % (cmdVerb,))
            self.done()
    
    def homeStart(self):
        """Start homing"""
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
        """Home third step: moved away from limit switch"""
        self.sendLine("Finding next full step")
        self.replyTimer.start(0.1, self.homeDone)
        
    def homeDone(self):
        """Homing finished"""
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
        for msgStr in [
            self.formatArr("%d", self.isHomed, "axis homed"),
            self.formatArr("%09d", self.cmdPos, "commanded position"),
            self.formatArr("%09d", self.measPos, "actual position"),
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
        self.measPos = newCmdPos + numpy.array(numpy.random.normal(0, 100, 6), dtype=int)

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
        self.resetUserNums()
        self.sendLine("OK")
    
    def sendLine(self, line):
        if Debug:
            print "sending: %r" % (line,)
        LineReceiver.sendLine(self, line)

def main(port):
    """Run a fake mirror controller Galil on the specified port"""
    #print "Starting fake Galil on port %s" % (port,)
    factory = protocol.ServerFactory()
    factory.protocol = FakeGalilProtocol
    reactor.listenTCP(port, factory)
    print "Starting fake Galil on port %s" % (port,)
    reactor.run()


# this only runs if the module was *not* imported
if __name__ == '__main__':
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    else:
        port = DefaultPort
    main(port=port)
