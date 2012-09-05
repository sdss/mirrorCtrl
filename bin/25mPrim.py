#!/usr/bin/env python
"""2.5m primary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir25mPrim import Mirror

UserPort = 2531

# for testing
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
