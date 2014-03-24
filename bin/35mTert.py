#!/usr/bin/env python2
"""3.5m tertiary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors import mir35mTert
import copy
import itertools

UserPort = 3532

GalilHost = "tccserv35m-p"
GalilPort = 3500 # port 15

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = mir35mTert,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
