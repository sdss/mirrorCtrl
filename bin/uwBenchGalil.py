#!/usr/bin/env python
"""Talk to a bench mirror with only 3 axes: same parameters as 3.5m tertiary
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir35mTert import Mirror

UserPort = 3532

# for testing
GalilHost = 'tccservdev.astro.washington.edu'
GalilPort = 2011

Mirror.name = 'UW Bench Galil (simple 3.5m tertiary)'

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
