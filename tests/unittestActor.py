#!/usr/bin/env python
import unittest
import Tkinter
import thread
import multiprocessing
import time
import telnetlib

import mirror
from data import genMirrors
import twistedGalil
    
UserPort = 1025
ControllerAddr = 'localhost'
ControllerPort = 8000
isOK = 0
Iter = 0

# 2.5m M2 because galil replies have 5 actuators
mirr = genMirrors.Sec25().makeMirror() 
# 3.5m M3 device because galil replies are specific to this mirror...
# ./data/rawGalilReplies.txt have examples of all galil replies
# these are spouted from twistedGalil.py
device = mirror.GalilDevice35M3

def doCmdTest(root, cmdStrs):
    """Loop over cmds in cmdStrs, send to actor via telnet, listen for ":".
    When all commands have been sent, destroy the actor instance, and the main
    test thread will continue and check the exit conditions of this thread.
    If isOK is 0, a test failed.
    """
    global isOK
    global Iter
    # time for widget to start up and initialize
    time.sleep(3)
    tnet = telnetlib.Telnet(host = ControllerAddr, port = UserPort, timeout=100)
    # wait for connection
    time.sleep(2)
    Iter = 0
    for ind, cmd in enumerate(cmdStrs):
        tnet.write(cmd) # send command to actor
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
    root.destroy()

def makeActor(dev, mirr):
    """set up actor, return the Tkinter root and the actor itself
    """
    root = Tkinter.Tk()
    actor = mirror.GalilActor(dev, mir=mirr, userPort=UserPort, controllerAddr=ControllerAddr, controllerPort = ControllerPort)
    #root.mainloop()
    return root, actor

class ActorTests(unittest.TestCase):
    """Tests for actor
    """
#     def testParser(self):
#         with open('./data/rawGalilReplies.txt', 'r') as f:
#             RawGalilReplies = f.readlines()
#         time.sleep(4) # wait for prev test to shut down...
#         root, actor = makeActor(device, mirr)
#         parser = actor.galilDev.parseLine
#         for line in RawGalilReplies:
#             out = parser(line)
#             print 'out: ', out
#         root.destroy()
#         self.assertEqual(0, 0, 'they are equal')
        
    def testCmds(self):
        """This test starts up the actor and twistedGalil, then sends commands
        to the actor via telnet, then listens for a ":" in the reply, signaling
        that the command finished properly.
        
        note: lots of sleep time to allow for startup and shutdown of actors,
        connections to ports, etc.
        """
        global isOK
        isOK = 1
        # start up the fake galil
        tGal = multiprocessing.Process(target=twistedGalil.main, args=())
        tGal.start()
        time.sleep(1)
        # set up actor
        root, actor = makeActor(device, mirr)
        # list of commands you want to test
        cmdStrs = [
            "move 4,4,4\r\n",
            "home A,B,C\r\n",
            "status\r\n",
            "showparams\r\n",
            "stop\r\n"
            ]
        # send commands from different thread
        # actor must run in main thread
        thread.start_new_thread(lambda: doCmdTest(root, cmdStrs), ())
        # start up the actor instance
        root.mainloop()
        tGal.terminate()
        self.assertEqual(1, isOK, 'failed on cmd %s' % cmdStrs[Iter])


if __name__ == '__main__':
    unittest.main()