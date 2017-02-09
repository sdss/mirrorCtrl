from __future__ import absolute_import, division
"""3.5m tertiary mirror controller
"""
import syslog

from mirrorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mTert

__all__ = ["Tert35mMirrorCtrl"]


GalilHost = "tccserv35m-p"
GalilPort = 3500 # terminal server port 15

class Tert35mMirrorCtrl(MirrorCtrl):
    Name = "tert35m"
    UserPort = 3533
    Facility = syslog.LOG_LOCAL2
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice(
                mirror = mir35mTert,
                host = GalilHost,
                port = GalilPort,
                maxIter = 5,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

