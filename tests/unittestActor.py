#!/usr/bin/env python
import unittest
import Tkinter

import mirror
from data import genMirrors

root = Tkinter.Tk()

with open('./data/rawGalilReplies.txt', 'r') as f:
    RawGalilReplies = f.readlines()

UserPort = 1025
ControllerAddr = 'localhost'
ControllerPort = 8000

mirList = [genMirrors.Prim25().makeMirror(), # defaults to AdjBase actuators.
           genMirrors.Prim25(actType='adjLen').makeMirror(), # AdjLen (old) type actuators.
           genMirrors.Prim25(fix=1).makeMirror(), 
           genMirrors.Prim25(fix=2).makeMirror(),
           genMirrors.Sec25().makeMirror(),
           genMirrors.Sec35().makeMirror(),
           genMirrors.Tert35().makeMirror(), # new non-inf length links
           genMirrors.Tert35(vers='old').makeMirror() # old semi-inf length links
           ]
device = mirror.GalilDevice35M3
actor = mirror.GalilActor(device, mir=mirList[-1], userPort=UserPort, controllerAddr=ControllerAddr, controllerPort = ControllerPort)

root.mainloop()
# tests
class ActorTests(unittest.TestCase):
    """Tests for mirrors
    """
    def testParser(self):
        parser = device(ControllerAddr, ControllerPort).parseLine
        for line in RawGalilReplies:
            out = parser(line)
            print out
        self.assertEqual(0, 0, 'they are equal')

if __name__ == '__main__':
    unittest.main()