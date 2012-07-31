#!/usr/bin/env python
"""
This communicates with the Galil Actor.  It pings semi-faked Galil replies back for testing.
"""
Port = 8000 # must match device port in Galil Actor.

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

from twisted.internet import reactor, protocol, defer
from twisted.protocols.basic import LineReceiver
import re
from RO.Comm.TwistedTimer import Timer

Debug = True

ReplyDict = dict(
    move = """0020.7,  0021.0,  0021.0,  0003.5,  0003.4 max sec for move
           -001192321,  000648641, -000475989,  000014310, -000000017 target position
           -001194727,  000649299, -000475794,  000014384,  000000041 final position
           OK""",

    home = """: 0362.7,  0362.7,  0362.7,  0000.0,  0000.0 max sec to find reverse limit switch
            Reverse limit switch not depressed for axes:  0,  1,  0,  0,  0 - trying again
             0000.2,  0362.7,  0000.2,  0000.0,  0000.0 max sec to find reverse limit switch
             0300.1,  0300.1,  0300.1,  0000.0,  0000.0 max sec to find home switch
             0008.2,  0008.2,  0008.2,  0000.0,  0000.0 sec to move away from home switch
            Finding next full step
             041,  006.6 microsteps, sec to find full step
            -000006732,  000014944,  000003741,  999999999,  999999999 position error
             1,  1,  1,  0,  0 axis homed
            -007250000, -007250000, -007250000,  999999999,  999999999 commanded position
            -007249995, -007249995, -007249982,  999999999,  999999999 actual position
             02.10, 5 software version, NAXES number of axes
             1, 1, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections
             00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME
            -007250000, -007250000, -007250000,  000000000, -002510000 
             007250000,  007250000,  007250000,  000000000,  002510000 RNGx/2 forward limits
             000050000,  000050000,  000050000,  000050000,  000082000 SPDx speed
             000005000,  000005000,  000005000,  000005000,  000004000 HMSPDx homing speed
             000500000,  000500000,  000500000,  000500000,  000164000 ACCx acceleration
             000000000,  000000000,  000000000,  000000000,  000000000 MINCORRx min correction
             001000000,  001000000,  001000000,  000000000,  000074000 MAXCORRx max correction
             000000050,  000000050,  000000050,  000000050,  000000001 ST_FSx microsteps/full step
             000400000,  000400000,  000400000,  000000000,  000041000 MARGx dist betw hard & soft rev lim
             000000000,  000000000,  000000000,  000000000,  000000000 INDSEP index encoder pulse separation
            -0003.1496, -0003.1496, -0003.1496,  0000.0000,  0095.5733 ENCRESx encoder resolution (microsteps/tick)
             02.00 version of M3-specific additions
             01 000007000 off-on-error?, error limit for tertiary rotation
             06.0 05.0 00.1 time to close rotation clamp, open clamp, turn on at-slot sensor (sec)
             15.1 00.5 02.0 max time, poll time, addtl run time for primary mirror cover motion (sec)
             10.0 time for primary mirror eyelid motion (sec)
             0000.0,  0000.0,  0000.0,  0000.0,  0000.0 sec to finish move
            OK""",

    status = """1,  1,  1,  1,  1 axis homed
        -001176643,  000679110, -000447314,  000014042, -000000129 commanded position
        -001192236,  000648745, -000475977,  000014310, -000000017 actual position
        00008196,  00008196,  00008196,  00008196,  00008196 status word
        OK""",

    showpar = """02.10, 5 software version, NAXES number of axes
        1, 0, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections
        00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME
        -007250000, -007250000, -007250000, -000095000, -000095000 -RNGx/2 reverse limits
        007250000,  007250000,  007250000,  000095000,  000095000 RNGx/2 forward limits
        000050000,  000050000,  000050000,  000005000,  000005000 SPDx speed
        000005000,  000005000,  000005000,  000000500,  000000500 HMSPDx homing speed
        000500000,  000500000,  000500000,  000050000,  000050000 ACCx acceleration
        000000000,  000000000,  000000000,  000000000,  000000000 MINCORRx min correction
        001000000,  001000000,  001000000,  000015000,  000015000 MAXCORRx max correction
        000000050,  000000050,  000000050,  000000050,  000000050 ST_FSx microsteps/full step
        000400000,  000400000,  000400000,  000005000,  000005000 MARGx dist betw hard & soft rev lim
        000000000,  000000000,  000000000,  000000000,  000000000 INDSEP index encoder pulse separation
        -0003.1496, -0003.1496, -0003.1496,  0001.5750,  0001.5750 ENCRESx encoder resolution (microsteps/tick)
        OK""",
)

class SpitBack(LineReceiver):
    """Lines..."""
    def __init__(self, *args, **kwargs):
        self.replyTimer = Timer()
        self.replyList = None
    
    def echo(self, line):
        self.transport.write(line + ":")

    def lineReceived(self, line):
        """As soon as any data is received, look at it and write something back."""
        if Debug:
            print "received: %r" % (line,)
        line = line.lower()
        cmdList = [c.strip() for c in line.split(";") if c.strip()]
        for cmd in cmdList:
            self.processCmd(cmd)
        
    def processCmd(self, cmd):
        if Debug:
            print "processCmd(cmd=%r); replyList=%r" % (cmd, self.replyList)
        self.echo(cmd)
        if cmd in ("st", "rs"):
            self.replyList = None
            return

        if self.replyList:
            # Reject new command; but what does the Galil really do?
            # It probably accepts non-XQ commands such as A=value, but what about XQ commands?
            self.sendLine("? Busy")
            self.sendLine("OK")
            return
            
        cmdMatch = re.match(r"xq *#([a-z]+)", cmd)
        if not cmdMatch:
            return

        cmdVerb = cmdMatch.groups()[0]

        try:
            reply = ReplyDict[cmdVerb]
        except Exception:
            self.sendLine("? Unrecognized command %s" % (cmdVerb,))
            self.sendLine("OK")
            return
        
        self.replyList = [""] + [r.strip() for r in reply.split("\n") if r.strip()]
        self.replyTimer.start(0.001, self.writeBack)
    
    def sendLine(self, line):
        if Debug:
            print "sending: %r" % (line,)
        LineReceiver.sendLine(self, line)
        
    def writeBack(self):
        if not self.replyList:
            return
        
        nextReply = self.replyList.pop(0)
        self.sendLine(nextReply)
        if self.replyList:
            self.replyTimer.start(0.25, self.writeBack)

def main():
    """Run a fake mirror controller Galil on port Port"""
    print "Starting fake Galil on port %s" % (Port,)
    factory = protocol.ServerFactory()
    factory.protocol = SpitBack
    reactor.listenTCP(Port, factory) # Port defined up top
    #reactor.connectTCP('localhost', Port, factory, timeout=1)
    reactor.run()


# this only runs if the module was *not* imported
if __name__ == '__main__':
    main()
