#!/usr/bin/env python2
"""3.5m tertiary mirror controller
"""
import os.path

import mirrorCtrl
from mirrorCtrl.mirrors import mir35mTert

UserPort = 3533

GalilHost = "tccserv35m-p"
GalilPort = 3500 # port 15

name = os.path.splitext(os.path.basename(__file__))[0]
device = mirrorCtrl.GalilDevice(
    name = name,
    mirror = mir35mTert,
    host = GalilHost,
    port = GalilPort,
)
mirrorCtrl.runMirrorCtrl(
    name = name,
    device = device,
    userPort = UserPort,
)
