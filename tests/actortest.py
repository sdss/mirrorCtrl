#!/usr/bin/env python
"""starts up the galil actor, allowing for testing
Intended for communication via telnet"""

import numpy
import mirror

from data import genMirrors
UserPort = 1025
ControllerAddr = 'localhost'
ControllerPort = 8000 # must match in fakeGalil.py for testing

# .makeMirror takes an mirId arguement wich is used in the instrument ICC to tailor functionality
# between different mirrors
Mir = genMirrors.Sec25().makeMirror(name='Tert') # this matches hard coded (faked) replies from fakeGalil

if __name__ == "__main__":
    mirror.runGalil(Mir, mirror.GalilDevice35M3, UserPort, ControllerAddr, ControllerPort)
