from __future__ import absolute_import, division
"""SDSS 2.5m primary mirror controller
"""
import syslog

from mirrorCtrl import GalilDevice25Prim, MirrorCtrl
from mirrorCtrl.mirrors import mir25mPrim

__all__ = ["Prim25mMirrorCtrl"]

GalilHost = "t-g-sdss-2"
GalilPort = 2900 # terminal server port 9

class Prim25mMirrorCtrl(MirrorCtrl):
    Name = "prim25m"
    UserPort = 2531
    Facility = syslog.LOG_LOCAL2
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice25Prim(
                mirror = mir25mPrim,
                host = GalilHost,
                port = GalilPort,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

