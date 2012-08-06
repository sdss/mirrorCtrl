#!/usr/bin/env python
import unittest
import Tkinter
import thread
import multiprocessing
import time
import telnetlib

import mirrorCtrl
from data import genMirrors
import fakeGalil
    
UserPort = 1025
GalilHost = 'localhost'
GalilPort = 8001
isOK = 0
Iter = 0

def doCmdTest(reactor, cmdStrs):
    """Loop over cmds in cmdStrs, send to actor via telnet, listen for ":".
    When all commands have been sent, destroy the actor instance, and the main
    test thread will continue and check the exit conditions of this thread.
    If isOK is 0, a test failed.
    """
    global isOK
    global Iter
    # time for widget to start up and initialize
    time.sleep(3)
    tnet = telnetlib.Telnet(host = GalilHost, port = UserPort, timeout=100)
    # wait for connection
    time.sleep(2)
    Iter = 0
    for ind, cmd in enumerate(cmdStrs):
        tnet.write(cmd + "\r\n") # send command to actor
        # listen until ":" is received or timeout
        justIn = tnet.read_until(":", 15)
        justIn = justIn.split('\r\n')
        if not (":" in justIn[-1]):
            # if ":" not found
            isOK = 0
            Iter = ind
            break        
    tnet.close()
    time.sleep(2)
    reactor.stop()

def makeActor():
    """Create a 3.5m secondary mirror controller actor
    
    Replies from the fakeGalil are appropriate for this mirror controller.
    """
# 
# # 2.5m M2 because galil replies have 5 actuators
# mirr = genMirrors.Sec25().makeMirror() 
# # 3.5m M3 device because galil replies are specific to this mirror...
# # ./data/rawGalilReplies.txt have examples of all galil replies
# # these are spouted from fakeGalil.py
# Device = mirror.GalilDevice35M3

    mirror = genMirrors.Sec25().makeMirror()
    device = mirrorCtrl.GalilDevice35M3(mirror=mirror, host=GalilHost, port=GalilPort)
    actor = mirrorCtrl.MirrorCtrl(device=device, userPort=UserPort)
    return actor

class ActorTests(unittest.TestCase):
    """Tests for actor
    """
#     def testParser(self):
#         with open('./data/rawGalilReplies.txt', 'r') as f:
#             RawGalilReplies = f.readlines()
#         from twisted.internet import reactor
#         time.sleep(4) # wait for prev test to shut down...
#         actor = makeActor()
#         parser = actor.galilDev.parseLine
#         for line in RawGalilReplies:
#             out = parser(line)
#             print 'out: ', out
#         reactor.stop()
#         self.assertEqual(0, 0, 'they are equal')
        
    def testCmds(self):
        """This test starts up the actor and fakeGalil, then sends commands
        to the actor via telnet, then listens for a ":" in the reply, signaling
        that the command finished properly.
        
        note: lots of sleep time to allow for startup and shutdown of actors,
        connections to ports, etc.
        """
        from twisted.internet import reactor
        global isOK
        isOK = 1
        # start up the fake galil
        tGal = multiprocessing.Process(target=fakeGalil.main, args=(GalilPort,))
        tGal.start()
        time.sleep(1)
        # set up actor
        actor = makeActor()
        # list of commands you want to test
        cmdStrs = [
            "move 4,4,4",
            "home A,B,C",
            "status",
            "showparams",
            "stop"
            ]
        # send commands from different thread
        # actor must run in main thread
        thread.start_new_thread(lambda: doCmdTest(reactor, cmdStrs), ())
        # start up the actor instance
        reactor.run()
        tGal.terminate()
        self.assertEqual(1, isOK, 'failed on cmd %s' % cmdStrs[Iter])


if __name__ == '__main__':
    unittest.main()
