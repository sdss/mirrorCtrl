#!/usr/bin/env python2
"""2.5m primary mirror controller
"""
import os.path

import mirrorCtrl
from mirrorCtrl.mirrors import mir25mPrim

UserPort = 2531

# for testing
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing

name = os.path.splitext(os.path.basename(__file__))[0]
device = mirrorCtrl.GalilDevice(
    name = name,
    mirror = mir25mPrim,
    host = GalilHost,
    port = GalilPort,
)
mirrorCtrl.runMirrorCtrl(
    name = name,
    device = device,
    userPort = UserPort,
)
