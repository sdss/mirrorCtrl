#!/usr/bin/env python2
from __future__ import absolute_import, division
"""Talk to a bench mirror with only 3 axes: same parameters as 3.5m tertiary
"""
from mirrorCtrl import GalilDevice, MirrorCtrl
from mirrorCtrl.mirrors import mir35mTert

# for testing
GalilHost = 'tccservdev.astro.washington.edu'
GalilPort = 2011

# testing using tunnel on 3500
# GalilHost = "localhost"
# GalilPort = 16000

class UWBenchMirrorCtrl(MirrorCtrl):
    Name = "uwBench"
    UserPort = 35320
    def __init__(self):
        MirrorCtrl.__init__(self,
            device = GalilDevice(
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

    startFileLogging("uwBenchGalil")
    UWBenchMirrorCtrl()
    reactor.run()