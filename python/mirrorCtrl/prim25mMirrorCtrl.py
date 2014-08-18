from __future__ import absolute_import, division
"""SDSS 2.5m primary mirror controller
"""
import syslog

from mirrorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir25mPrim

__all__ = ["Sec25mMirrorCtrl"]

GalilHost = "t-g-sdss-2"
GalilPort = 2900 # terminal server port 9

class Sec25mMirrorCtrl(MirrorCtrl):
    Name = "sec25m"
    UserPort = 2531
    Facility = syslog.LOG_LOCAL2
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice(
                name = self.Name,
                mirror = mir25mPrim,
                host = GalilHost,
                port = GalilPort,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

