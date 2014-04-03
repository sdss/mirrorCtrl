#!/usr/bin/env python2
"""3.5m secondary mirror controller
"""
import os.path

import mirrorCtrl
from mirrorCtrl.mirrors import mir35mSec

UserPort = 3532

GalilHost = "tccserv35m-p"
GalilPort = 2800 # port 8

name = os.path.splitext(os.path.basename(__file__))[0]
device = mirrorCtrl.GalilDevice(
    name = name,
    mirror = mir35mSec,
    host = GalilHost,
    port = GalilPort,
)
mirrorCtrl.runMirrorCtrl(
    name = name,
    device = device,
    userPort = UserPort,
)
