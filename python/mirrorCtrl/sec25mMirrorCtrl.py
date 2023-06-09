from __future__ import absolute_import, division
"""SDSS 2.5m secondary mirror controller
"""
import syslog

from mirrorCtrl import GalilDevice25Sec, MirrorCtrl
from mirrorCtrl.mirrors import mir25mSec

__all__ = ["Sec25mMirrorCtrl"]

#GalilHost = "t-g-sdss-2"
GalilHost = "10.25.1.11"
GalilPort = 3000 # terminal server port 10
# GalilHost = "localhost"

class Sec25mMirrorCtrl(MirrorCtrl):
    Name = "sec25m"
    UserPort = 2532
    Facility = syslog.LOG_LOCAL3
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice25Sec(
                mirror = mir25mSec,
                host = GalilHost,
                port = GalilPort,
                maxIter = 3,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

