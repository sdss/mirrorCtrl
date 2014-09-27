#!/usr/bin/env python2
from __future__ import absolute_import, division
"""Talk to a bench mirror with only 3 axes: same parameters as 3.5m tertiary
"""
import syslog

from mirrorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mTert

# UserPort = 3532

# for testing
# GalilHost = 'tccservdev.astro.washington.edu'
# GalilPort = 2011

# testing using tunnel on 3500
GalilHost = "localhost"
GalilPort = 16000

# Mirror.name = 'UW Bench Galil (simple 3.5m tertiary)'

class UWBenchMirrorCtrl(MirrorCtrl):
    Name = "uwBench"
    UserPort = 3532
    # Facility = syslog.LOG_LOCAL3
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice(
                name = self.Name,
                mirror = mir35mTert,
                host = GalilHost,
                port = GalilPort,
            ),
            userPort = self.UserPort,
            name = self.Name,
        )

if __name__ == "__main__":
    from twisted.internet import reactor
    from twistedActor import startFileLogging

    startFileLogging("/Users/csayres/Desktop/")
    UWBenchMirrorCtrl()
    reactor.run()