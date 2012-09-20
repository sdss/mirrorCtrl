#!/usr/bin/env python
"""2.5m secondary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir25mSec import Mirror

UserPort = 2532

# for testing
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice25Sec(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
