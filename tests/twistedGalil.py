#!/usr/bin/env python
"""
This communicates with the Galil Actor.  It pings semi-faked Galil replies back for testing.
"""
Port = 8000 # must match device port in Galil Actor.

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

from twisted.internet import reactor, protocol, defer
from twisted.protocols.basic import LineReceiver
import time
import collections

def genStrFromMove():
    """ gen galil reply from move command.  Same reply is always sent.
    
    Command was (from Russell): A=-001192321;: B=000648641;: C= -000475989;: D= 000014310;: E= -000000017
    XQ#MOVE"""
    
    str = "0020.7,  0021.0,  0021.0,  0003.5,  0003.4 max sec for move\r\n\
           -001192321,  000648641, -000475989,  000014310, -000000017 target position\r\n\
           -001194727,  000649299, -000475794,  000014384,  000000041 final position\r\n\
           OK" 
    return str
    
def genStrFromHome():
    """ from 3.5m tert """

    str = ": 0362.7,  0362.7,  0362.7,  0000.0,  0000.0 max sec to find reverse limit switch\r\n\
            Reverse limit switch not depressed for axes:  0,  1,  0,  0,  0 - trying again\r\n\
             0000.2,  0362.7,  0000.2,  0000.0,  0000.0 max sec to find reverse limit switch\r\n\
             0300.1,  0300.1,  0300.1,  0000.0,  0000.0 max sec to find home switch\r\n\
             0008.2,  0008.2,  0008.2,  0000.0,  0000.0 sec to move away from home switch\r\n\
            Finding next full step\r\n\
             041,  006.6 microsteps, sec to find full step\r\n\
            -000006732,  000014944,  000003741,  999999999,  999999999 position error\r\n\
             1,  1,  1,  0,  0 axis homed\r\n\
            -007250000, -007250000, -007250000,  999999999,  999999999 commanded position\r\n\
            -007249995, -007249995, -007249982,  999999999,  999999999 actual position\r\n\
             02.10, 5 software version, NAXES number of axes\r\n\
             1, 1, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections\r\n\
             00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME\r\n\
            -007250000, -007250000, -007250000,  000000000, -002510000 -RNGx/2 reverse limits\r\n\
             007250000,  007250000,  007250000,  000000000,  002510000 RNGx/2 forward limits\r\n\
             000050000,  000050000,  000050000,  000050000,  000082000 SPDx speed\r\n\
             000005000,  000005000,  000005000,  000005000,  000004000 HMSPDx homing speed\r\n\
             000500000,  000500000,  000500000,  000500000,  000164000 ACCx acceleration\r\n\
             000000000,  000000000,  000000000,  000000000,  000000000 MINCORRx min correction\r\n\
             001000000,  001000000,  001000000,  000000000,  000074000 MAXCORRx max correction\r\n\
             000000050,  000000050,  000000050,  000000050,  000000001 ST_FSx microsteps/full step\r\n\
             000400000,  000400000,  000400000,  000000000,  000041000 MARGx dist betw hard & soft rev lim\r\n\
             000000000,  000000000,  000000000,  000000000,  000000000 INDSEP index encoder pulse separation\r\n\
            -0003.1496, -0003.1496, -0003.1496,  0000.0000,  0095.5733 ENCRESx encoder resolution (microsteps/tick)\r\n\
             02.00 version of M3-specific additions\r\n\
             01 000007000 off-on-error?, error limit for tertiary rotation\r\n\
             06.0 05.0 00.1 time to close rotation clamp, open clamp, turn on at-slot sensor (sec)\r\n\
             15.1 00.5 02.0 max time, poll time, addtl run time for primary mirror cover motion (sec)\r\n\
             10.0 time for primary mirror eyelid motion (sec)\r\n\
             0000.0,  0000.0,  0000.0,  0000.0,  0000.0 sec to finish move\r\n\
            OK"
    return str

def genStrFromStatus():
    """ gen status reply from status command. Same reply is always sent.
    
    Command was XQ#Status.  Note: 3.5m M3 has additional lines of output that should be
    ignored by the Actor. As well as > 3 axes"""

    str = "1,  1,  1,  1,  1 axis homed\r\n\
        -001176643,  000679110, -000447314,  000014042, -000000129 commanded position\r\n\
        -001192236,  000648745, -000475977,  000014310, -000000017 actual position\r\n\
        00008196,  00008196,  00008196,  00008196,  00008196 status word\r\n\
        OK"
    return str
    
def genStrFromShowpar():
    """ gen reply from showpar command: XQ#SHOWPAR """

    str = "02.10, 5 software version, NAXES number of axes\r\n\
        1, 0, 01 DOAUX aux status? MOFF motors off when idle? NCORR # corrections\r\n\
        00.10, 00.00, 30.00 WTIME, ENCTIME, LSTIME\r\n\
        -007250000, -007250000, -007250000, -000095000, -000095000 -RNGx/2 reverse limits\r\n\
        007250000,  007250000,  007250000,  000095000,  000095000 RNGx/2 forward limits\r\n\
        000050000,  000050000,  000050000,  000005000,  000005000 SPDx speed\r\n\
        000005000,  000005000,  000005000,  000000500,  000000500 HMSPDx homing speed\r\n\
        000500000,  000500000,  000500000,  000050000,  000050000 ACCx acceleration\r\n\
        000000000,  000000000,  000000000,  000000000,  000000000 MINCORRx min correction\r\n\
        001000000,  001000000,  001000000,  000015000,  000015000 MAXCORRx max correction\r\n\
        000000050,  000000050,  000000050,  000000050,  000000050 ST_FSx microsteps/full step\r\n\
        000400000,  000400000,  000400000,  000005000,  000005000 MARGx dist betw hard & soft rev lim\r\n\
        000000000,  000000000,  000000000,  000000000,  000000000 INDSEP index encoder pulse separation\r\n\
        -0003.1496, -0003.1496, -0003.1496,  0001.5750,  0001.5750 ENCRESx encoder resolution (microsteps/tick)\r\n\
        OK"
    return str
    
def genStrFromSt():
    """ Not actual output"""

    #str = '? Stop Error!'
    str = ''
    return str

CmdsTup = collections.namedtuple("CmdList", ["move", "home", "status", "showpar", "st"])    
cmdRplList = [genStrFromMove(), genStrFromHome(), genStrFromStatus(), genStrFromShowpar(), genStrFromSt()]
validCmds = CmdsTup(*cmdRplList) # for matching replies to commands

class SpitBack(LineReceiver):
    """Lines..."""

    def lineReceived(self, line):
        """As soon as any data is received, look at it and write something back."""
        print 'Got a line!'
        
        splitLine = line.lower().split("xq #")
        cmdRec = splitLine[-1].strip("; ") # just get the cmd without any ';' or ' '.
        try:
            reply = getattr(validCmds, cmdRec)
        except:
            raise RuntimeError('Command not recognized: %s' % (cmdRec))
        if cmdRec == "st":
            #interrupt everything
            self.replyList = None
            self.ind = None
        self.replyList = reply.split('\r\n') # split reply into seperate lines
        self.replyList.insert(0, line) # for echoing back the received cmd.
        self.ind = 0     
        reactor.callLater(1, self.writeBack)
        
    def writeBack(self):
        print 'Sending Line:', self.replyList[self.ind]
        if self.replyList != None: # to allow interrupt
            self.sendLine(self.replyList[self.ind]) # send one line at a time
            time.sleep(2) # pause inbetween lines sent, for the hell of it.
            self.ind += 1
            if self.ind <= len(self.replyList):
                # do another iter
                reactor.callLater(1, self.writeBack)
            else:
                return

def main():
    """This runs the protocol on specified port"""
    factory = protocol.ServerFactory()
    factory.protocol = SpitBack
    reactor.listenTCP(Port,factory) # Port defined up top
    #reactor.connectTCP('localhost', Port, factory, timeout=1)
    reactor.run()


# this only runs if the module was *not* imported
if __name__ == '__main__':
    main()