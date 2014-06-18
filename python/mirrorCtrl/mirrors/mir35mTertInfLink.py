#!/usr/bin/env python2
"""
Configuration of tertiary (flat) mirror for 3.5m APO telescope

This model uses the old-style infinite links, and models the mirror using 6 adjustable actuators.
Actuators ABC are in locations match that of mir.dat.  Ghost-actuators (DEF) are of infinite
length.

Acutators = Encoders here.  This mirror should behave exactly how the old tert did.
Note that only mount positions for ABC are relevant (afterall there are no other
actuators/encoders on the actual mirror).

This isn't true in real life.
It is expected that piston = trans y in all commands (as was true in the old style)
"""
__all__ = ["mir35mTertInfLink"]

import numpy
import mirrorCtrl
from mirrorCtrl.const import MMPerInch, RadPerDeg

## Mirror name
Name = 'mir35mTertInfLink'

def _makeMirror():
    """Create a 3.5m Tertiary Mirror, model using infinite links
    """


# from mir_35m.dat
    mirPosAct = numpy.asarray([[      0.,  257.673, -257.673, 0., 0., 298],
                            [-226.105,   89.479,   89.479, 0., 0.,  0.],
                            [ 194.674, -120.910, -120.910, 0., 0.,  0.]])

    basePosAct = numpy.asarray([[     0., 257.673,  -257.673,     1e+09,    -1e+09,       298],
                            [-271.006,  44.578,    44.578,  7.07e+08,  7.07e+08,  7.07e+08],
                            [ 149.773, -165.811, -165.811, -7.07e+08, -7.07e+08, -7.07e+08]])


    # Limits of motion (mount units) for each actuator
    ActMinMount = numpy.array([-7250000., -7250000., -7250000, -numpy.inf, -numpy.inf, -numpy.inf])
    ActMaxMount = numpy.array([ 7250000.,  7250000.,  7250000,  numpy.inf,  numpy.inf,  numpy.inf])
    # Offset (mount units) and scale (mount units/um) for each actuator
    ActMountOffset = numpy.array([      0.,       0.,       0.,       0.,       0.,        0])
    ActMountScale  = numpy.array([1259.843, 1259.843, 1259.843, 1259.843, 1259.843, 1259.843])

    actuatorList = []
    encoderList = []
    fixedLinkList = []
    # generate list of actuators, encoders, and fixed links
    for i in range(6):
        actuatorList.append(
            mirrorCtrl.AdjLengthLink(
            #mirrorCtrl.AdjBaseActuator(
                basePosAct[:, i], mirPosAct[:, i], ActMinMount[i],
                ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )
        encoderList.append(
            mirrorCtrl.AdjLengthLink(
                basePosAct[:, i], mirPosAct[:, i], ActMinMount[i],
                ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )
    # minCorrList = [4.0e-5]*3 # min correction (mm); 50 actuator microsteps
    # maxCorrList = [0.79]*3   # max correction (mm); 1000000 actuator microsteps

    minCorrList = [50]*6 # min correction actuator microsteps
    maxCorrList = [1000000]*6   # max correction actuator microsteps


    return mirrorCtrl.DirectMirror(
        actuatorList = actuatorList,
        fixedLinkList = fixedLinkList,
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

## The 3.5m Tertiary mirror construction
mir35mTertInfLink = _makeMirror()
