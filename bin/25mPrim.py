#!/usr/bin/env python2
"""2.5m primary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors import mir25mPrim

UserPort = 2531

# for testing
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = mir25mPrim,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
