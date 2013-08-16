#!/usr/bin/env python
"""3.5m secondary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir35mSec import Mirror

UserPort = 3532

UseFakeGalil = True

if UseFakeGalil:
    # for testing
    GalilHost = 'localhost'
    GalilPort = 8000 # matches fakeGalil.py for testing
else:
    GalilHost = "tccserv35m-p"
    GalilPort = 2800 # port 8

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
