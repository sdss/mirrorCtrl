#!/usr/bin/env python2
"""3.5m secondary mirror controller
"""
import syslog

from .mirorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mSec

__all__ = ["Sec35mMirrorCtrl"]

Name = "sec35m"
UserPort = 3532
Facility = syslog.LOG_LOCAL3

GalilHost = "tccserv35m-p"
GalilPort = 2800 # terminal server port 8

device = GalilDevice(
    name = Name,
    mirror = mir35mSec,
    host = GalilHost,
    port = GalilPort,
)

class Sec35mMirrorCtrl(MirrorCtrl):
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = device,
            userPort = UserPort,
            name = Name,
        )

