#!/usr/bin/env python
"""
This communicates with the Galil Actor.  It pings semi-faked Galil replies back for testing.
"""
Port = 8000 # must match device port in Galil Actor.

# Copyright (c) Twisted Matrix Laboratories.
# See LICENSE for details.

from twisted.internet import reactor, protocol
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
    """ don't have an example of this yet """

    str = "OK"
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

    str = '? Stop Error!'
    return str

CmdsTup = collections.namedtuple("CmdList", ["move", "home", "status", "showpar", "st"])    
cmdRplList = [genStrFromMove(), genStrFromHome(), genStrFromStatus(), genStrFromShowpar(), genStrFromSt()]
validCmds = CmdsTup(*cmdRplList) # for matching replies to commands

class SpitBack(LineReceiver):
    """Lines..."""
    
    def lineReceived(self, line):
        """As soon as any data is received, look at it and write something back."""
        splitLine = line.lower().split("xq #")
        cmdRec = splitLine[-1].strip("; ") # just get the cmd without any ';' or ' '.
        try:
            reply = getattr(validCmds, cmdRec)
        except:
            raise RuntimeError('Command not recognized: %s' % (cmdRec))
        replyList = reply.split('\r\n') # split reply into seperate lines
        for line in replyList:
            self.sendLine(line) # send back to Actor one line at a time
            time.sleep(.2) # pause inbetween lines sent, for the hell of it.


def main():
    """This runs the protocol on specified port"""
    factory = protocol.ServerFactory()
    factory.protocol = SpitBack
    reactor.listenTCP(Port,factory) # Port defined up top
    reactor.run()


# this only runs if the module was *not* imported
if __name__ == '__main__':
    main()