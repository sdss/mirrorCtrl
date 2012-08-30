#!/usr/bin/env python
"""starts up the galil actor, allowing for testing
Intended for communication via telnet"""

import numpy
import mirror
import sys
from data import genMirrors
UserPort = 1025
ControllerAddr = 'localhost'
ControllerPort = 8000 # must match in twistedGalil.py for testing

if (len(sys.argv) > 1):
    # ports were specified via command line
    UserPort = int(sys.argv[1])
    try:
        ControllerPort = int(sys.argv[2])
    except:
        pass
        
# .makeMirror takes an mirId arguement wich is used in the instrument ICC to tailor functionality
# between different mirrors
Mir = genMirrors.Sec25().makeMirror(name='Tert') # this matches hard coded (faked) replies from twistedGalil

if __name__ == "__main__":
    mirror.runGalil(Mir, mirror.GalilDevice35M3, UserPort, ControllerAddr, ControllerPort)