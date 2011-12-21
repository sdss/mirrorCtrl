#!/usr/bin/env python
"""starts up the galil actor, allowing for testing"""

import numpy
import mirror

from data import genMirrors
UserPort = 1025
Mir = genMirrors.Sec25().makeMirror() # this matches hard coded (faked) replies from twistedGalil

if __name__ == "__main__":
    mirror.runGalil(Mir, UserPort)