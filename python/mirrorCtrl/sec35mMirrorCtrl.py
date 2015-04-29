from __future__ import absolute_import, division
"""3.5m secondary mirror controller
"""
import syslog

from mirrorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mSec

__all__ = ["Sec35mMirrorCtrl"]

GalilHost = "tccserv35m-p"
GalilPort = 2800 # terminal server port 8

class Sec35mMirrorCtrl(MirrorCtrl):
    Name = "sec35m"
    UserPort = 3532
    Facility = syslog.LOG_LOCAL3
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice(
                mirror = mir35mSec,
                host = GalilHost,
                port = GalilPort,
                maxIter = 3,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

