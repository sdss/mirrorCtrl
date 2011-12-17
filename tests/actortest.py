#!/usr/bin/env python
"""starts up the galil actor"""

import numpy
import mirror

from data import genMirrors
UserPort = 1025
Mir = genMirrors.Prim25().makeMirror()

if __name__ == "__main__":
    mirror.runGalil(Mir, UserPort)