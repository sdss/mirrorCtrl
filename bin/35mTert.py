#!/usr/bin/env python
"""3.5m tertiary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir35mTert import Mirror

UserPort = 3532
UseFakeGalil = True

if UseFakeGalil:
    # for testing
    GalilHost = 'localhost'
    GalilPort = 8000 # matches fakeGalil.py for testing
else:
    GalilHost = "tccserv35m-p"
    GalilPort = 3500 # port 15

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
