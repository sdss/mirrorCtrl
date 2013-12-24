#!/usr/bin/env python
"""3.5m tertiary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.mirrors.mir35mTert import Mirror
import copy
import itertools

def getActEqEncMir(mirror):
    """ Returns the same mirror as input, except actuators are moved to be exactly aligned with actuators
    @param[in] mirror: a MirrorBase instance
    @return a mirror instance with moved actuators
    """
    mirror = copy.deepcopy(mirror)
    for act, enc in itertools.izip(mirror.actuatorList, mirror.encoderList):
        act.mirPos = enc.mirPos[:]
        act.basePos = enc.basePos[:]
    return mirror

Mirror = getActEqEncMir(Mirror)  # place actuators equal to encoders

UserPort = 3532

GalilHost = "tccserv35m-p"
GalilPort = 3500 # port 15

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
