#!/usr/bin/env python2
"""2.5m secondary mirror controller
"""
import os.path

import mirrorCtrl
from mirrorCtrl.mirrors import mir25mSec

UserPort = 2532

# for testing
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing

name = os.path.splitext(os.path.basename(__file__))[0]
device = mirrorCtrl.GalilDevice25Sec(
    name = name,
    mirror = mir25mSec,
    host = GalilHost,
    port = GalilPort,
)
mirrorCtrl.runMirrorCtrl(
    name = name,
    device = device,
    userPort = UserPort,
)
