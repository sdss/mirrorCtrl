#!/usr/bin/env python2
"""3.5m tertiary mirror controller
"""
import syslog

from .mirorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mTert

__all__ = ["Tert35mMirrorCtrl"]

Name = "tert35m"
UserPort = 3533
Facility = syslog.LOG_LOCAL2

GalilHost = "tccserv35m-p"
GalilPort = 3500 # terminal server port 15

device = GalilDevice(
    name = Name,
    mirror = mir35mTert,
    host = GalilHost,
    port = GalilPort,
)

class Tert35mMirrorCtrl(MirrorCtrl):
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = device,
            userPort = UserPort,
            name = Name,
        )

