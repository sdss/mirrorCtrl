#!/usr/bin/env python2
"""Configuration of secondary mirror for 3.5m APO telescope

This mirror model uses actuators = encoders, and mirror positions as defined by mir.dat
"""
__all__ = ["mir35mSecOldModel"]

import math
import numpy
import mirrorCtrl
from mirrorCtrl.const import MMPerInch, RadPerDeg

## Mirror name
Name = 'mir35mSecOldModel'
## Radial position of ABC actuators relative to mirror vertex
# actRad =   9.08 * MMPerInch
# ## Radial position of ABC encoders relative to mirror vertex
# encRad = 10.25 * MMPerInch

def _makeMirror():
    """Create a 3.5m Secondary Mirror
    """
    # from mir.dat
    """mir position x coordinate for each axis"""
    ActMirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
    """mir position y coordinate for each axis"""
    ActMirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
    """mir position z coordinate for each axis"""
    ActMirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
    """base position x coordinate for each axis"""
    ActBaseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
    """base position y coordinate for each axis"""
    ActBaseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
    """base position z coordinate for each axis"""
    ActBaseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])


    # Actuators are: Axial A, B, C, Transverse D, E
    # Limits of motion (mount units) for each actuator

    ActMinMount = numpy.array([-7250000., -7250000., -7250000., -95000., -95000])
    ActMaxMount = numpy.array([ 7250000.,  7250000.,  7250000.,  95000.,  95000])

    ActMountOffset = numpy.array([      0.,       0.,       0.,     0.,     0.])
    ActMountScale  = numpy.array([1259.843, 1259.843, 1259.843, 31.496, 31.496])

    # All distances are in mm, all angles are in degrees
    # All numbers are for the 3.5m secondary mirror support system
    # X = right, Y = up, Z = from the sky towards the telescope
    # where up/down/left/right are as seen with the telescope at the horizon,
    # standing behind the primary mirror, looking towards the secondary mirrorCtrl.

    mirAct = numpy.zeros([5,3])
    baseAct = numpy.zeros([5,3])
    mirEnc = numpy.zeros([5,3])
    baseEnc = numpy.zeros([5,3])

    # generate lists of link objects for mirror configuration
    actuatorList = []
    encoderList = []
    for i in range(5):
        ############## from mir.dat ################
        basePosAct = [ActBaseX[i], ActBaseY[i], ActBaseZ[i]]
        mirPosAct = [ActMirX[i], ActMirY[i], ActMirZ[i]]
        basePosEnc = basePosAct[:]
        mirPosEnc = mirPosAct[:]

        actuatorList.append(
            mirrorCtrl.AdjBaseActuator(
                basePosAct, mirPosAct,
                ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )
        encoderList.append(
            mirrorCtrl.AdjLengthLink(
                basePosEnc, mirPosEnc,
                ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )


    # FixedLengthLink
    # located between C and B on edge of mirror opposite of A.
    linkLength = 12.36 * MMPerInch # measured
    mirRadius = 1000. # guess
    zMirAx =  -152.806 # for Axial
    fixMirPos = numpy.array([0., -1*mirRadius, zMirAx]) # opposite of A
    fixBasePos = numpy.array([linkLength, -1*mirRadius, zMirAx])
    fixedLinkList = [mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos)]

    # minCorrList = [4.0e-5]*3 + [0.0016]*2 # min correction (mm); 50 actuator microsteps for all actuators
    # maxCorrList = [0.79]*3   + [0.16]*2 # max correction (mm); 1000000 actuator microsteps for A-C; 5000 for D-E

    minCorrList = [50]*5 # min correction (microsteps)
    maxCorrList = [1000000]*3   + [5000]*2 # max correction (microsteps)


    return mirrorCtrl.DirectMirror(
        actuatorList = actuatorList,
        fixedLinkList = fixedLinkList,
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

mir35mSecOldModel = _makeMirror()
