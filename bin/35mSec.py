#!/usr/bin/env python
"""3.5m secondary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors import mir35mSec

UserPort = 3532

GalilHost = "tccserv35m-p"
GalilPort = 2800 # port 8

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = mir35mSec,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
