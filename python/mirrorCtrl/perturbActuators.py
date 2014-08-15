"""Functions that adjust actuator positions on pre-constructed mirrors.  Mainly helpful for testing and emulating.
"""
import copy
import itertools
import math
import numpy

def getActEqEncMir(mirror):
    """ Returns the same mirror as input, except actuators are moved to be exactly aligned with actuators
    @param[in] mirror  a MirrorBase instance
    @return a mirror instance with moved actuators
    """
    mirror = copy.deepcopy(mirror)
    for act, enc in itertools.izip(mirror.actuatorList, mirror.encoderList):
        act.mirPos = enc.mirPos[:]
        act.basePos = enc.basePos[:]
    return mirror

def getActRandMove(mirror, seed=10):
    """ Apply a random xy translation to ABC actuators,
        Apply a random z translation to DE actuators
    """
    enc0BasePos = mirror.encoderList[0].basePos
    encRad = math.hypot(enc0BasePos[0], enc0BasePos[1])
    act0BasePos = mirror.actuatorList[0].basePos
    actRad = math.hypot(act0BasePos[0], act0BasePos[1])
    numpy.random.seed(seed)
    mirror = copy.deepcopy(mirror)
    lengthScale = numpy.abs(actRad - encRad)*2.
    for act in mirror.actuatorList[:3]:
        # let offsets vary by magnitude of true offset in any direction
        xOff, yOff = numpy.random.sample(2)*2.*lengthScale - lengthScale
        offset = numpy.asarray([xOff, yOff, 0.])
        act.mirPos += offset
        act.basePos += offset
    for act in mirror.actuatorList[3:]:
        # let offset vary by magnitude of true offset in z direction
        zOff = numpy.random.sample()*2.*lengthScale*0.5 - lengthScale*0.5
        offset = numpy.asarray([0., 0., zOff])
        act.mirPos += offset
        act.basePos += offset
    return mirror