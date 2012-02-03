#!/usr/bin/env python
"""starts up the galil actor, allowing for testing"""

import numpy
import mirror

from data import genMirrors
UserPort = 1025

# .makeMirror takes an mirId arguement wich is used in the instrument ICC to tailor functionality
# between different mirrors
Mir = genMirrors.Sec25().makeMirror('2.5m M2') # this matches hard coded (faked) replies from twistedGalil

if __name__ == "__main__":
    mirror.runGalil(Mir, UserPort)