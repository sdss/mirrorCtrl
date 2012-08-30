#!/usr/bin/env python
"""3.5m tertiary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.devices.device35mTert import Mirror

UserPort = 3532

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
