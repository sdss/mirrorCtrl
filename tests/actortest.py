#!/usr/bin/env python
"""starts up the galil actor, allowing for testing
Intended for communication via telnet"""

import numpy
import mirrorCtrl
from mirrorCtrl.mirrors.mir35mTert import Mirror
import sys
from data import genMirrors
UserPort = 1025
ControllerAddr = 'tccservdev.astro.washington.edu'
ControllerPort = 2011

if (len(sys.argv) > 1):
    # ports were specified via command line
    UserPort = int(sys.argv[1])
    try:
        ControllerPort = int(sys.argv[2])
    except:
        pass
        
# .makeMirror takes an mirId arguement wich is used in the instrument ICC to tailor functionality
# between different mirrors

mirDev = mirrorCtrl.GalilDevice35Tert(Mirror, ControllerAddr, ControllerPort)
if __name__ == "__main__":
    #mirrorCtrl.runMirrorCtrl(Mir, mirrorCtrl.GalilDevice35Tert, UserPort, ControllerAddr, ControllerPort)
    mirrorCtrl.runMirrorCtrl(mirDev, UserPort)
