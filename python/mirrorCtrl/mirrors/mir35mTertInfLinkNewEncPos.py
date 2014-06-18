#!/usr/bin/env python2
"""Configuration of tertiary (flat) mirror for 3.5m APO telescope

This model uses the old-style infinite links, and models the mirror using 6 adjustable actuators.
Actuators ABC are in locations match that of mir.dat.  Ghost-actuators (DEF) are of infinite
length.

Acutators = Encoders here --BUT--  Actuator/Encoder locations (mir/base pos) were determined
by copying (printed) output of the encoder positions from mir35mTert after rotation into
the correct coordinate system.  They are hardcoded
below.  The construction of this mirror was used to perform a quick test.  It should
probably be ditched in the future, but I'm keeping it around until we are sure
the mirrors are working how we want...

This isn't true in real life.
It is expected that piston = trans y in all commands (as was true in the old style)
"""
__all__ = ["mir35mTertInfLinkNewEncPos"]

import numpy
import mirrorCtrl
from mirrorCtrl.const import MMPerInch, RadPerDeg

## Mirror name
Name = 'mir35mTertInfLinkNewEncPos'

def _makeMirror():
    """Create a 3.5m Tertiary Mirror, model using infinite links
    """


# from mir_35m.dat
    mirPosAct = numpy.asarray([[      0.,  235.148, -235.148, 0., 0., 298],
                            [-207.713,   80.283,   80.283, 0., 0.,  0.],
                            [ 176.282, -111.714, -111.714, 0., 0.,  0.]])

    basePosAct = numpy.asarray([[     0., 235.148,  -235.148,     1e+09,    -1e+09,       298],
                            [-252.615,  35.382,    35.382,  7.07e+08,  7.07e+08,  7.07e+08],
                            [ 131.381, -156.616, -156.616, -7.07e+08, -7.07e+08, -7.07e+08]])


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
mir35mTertInfLinkNewEncPos = _makeMirror()
