#!/usr/bin/env python2
"""
This is a copy of mir25mSec, with all encoders set equal to actuators, and allowing the
fixed link to behave as and adjustable link.  This is to reproduce the behavior of the old tcc's fitter
(no induced rotations/tilts/translations).  This mirror is necessary for converting collimation coefficients
between the old and new tcc during commissioning

"""
__all__ = ["mir25mSecOldModel"]

import numpy

import mirrorCtrl
from mirrorCtrl.const import MMPerInch

## Mirror Name
Name = 'mir25mSecOldModel'

def _makeMirror():
    """Construct a 2.5m Secondary mirror
    """

    # Actuators are: Axial A, B, C, Transverse A, B
    # Limits of motion (mount units) for each actuator
    ActMinMount = numpy.array([-7250000., -7250000., -7250000., -18000., -18000])
    ActMaxMount = numpy.array([ 7250000.,  7250000.,  7250000.,  18000.,  18000])

    # Offset (mount units) and scale (mount units/um) for each actuator
    ActMountOffset = numpy.array([     0.,      0.,      0.,  1700., -1700.])
    ActMountScale  = numpy.array([1259.84, 1259.84, 1259.84, 31.496, 31.496])

    # Position of each end of each actuator (mm), relative to mirror vertex
    # The offset that puts the system into collimation is probably due to two effects:
    # - offset of base position (as with M1 and handle the same way)
    # - misalignment of the secondary frame; this is handled as an orientation
    #   offset in the instrument blocks, not in mir.dat.
    # One way to separate these two effects is to measure the offset of the
    # base position with respect to the frame and assign anything else to
    # misalignment of the secondary frame.
    ActMirX  = numpy.array([ 293.81, -233.08,  -60.73,   19.80,  -19.80])
    ActMirY  = numpy.array([  99.51,  204.69, -304.20,  -19.80,  -19.80])
    ActMirZ  = numpy.array([-193.00, -193.00, -193.00, -263.80, -263.80])
    ActBaseX = numpy.array([ 293.81, -233.08,  -60.73,   56.57,  -56.57])
    ActBaseY = numpy.array([  99.51,  204.69, -304.20,  -56.57,  -56.57])
    ActBaseZ = numpy.array([-280.00, -280.00, -280.00, -263.80, -263.80])

    # z position of mirror and base gimbals of central linear bearing (mm)
    CtrMirZ = -135.70
    CtrBaseZ = -178.40

    # generate a lists of link objects for mirror configuration
    actuatorList = []
    encoderList = []
    for i in range(5):
        baseAct = numpy.array([ActBaseX[i], ActBaseY[i], ActBaseZ[i]])
        mirAct = numpy.array([ActMirX[i], ActMirY[i], ActMirZ[i]])
        # actuatorList.append(
        #     mirrorCtrl.AdjBaseActuator(
        #         baseAct, mirAct, ActMinMount[i],
        #         ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
        #     )
        # )
        link = mirrorCtrl.AdjLengthLink(
            baseAct, mirAct, ActMinMount[i],
            ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
        )
        actuatorList.append(link)
        encoderList.append(link)
    # Fixed Link from French's anti-rotation arm drawings, and speaking with him.
    # modify to be adjustable (to reproduce fitting behavior of old tcc)

    # note: French's model has Z = 1 inch below glass, but I'm adopting Z = actuators' mir pos.
    fixMirPos = numpy.array([0., -17.296 * MMPerInch, -193.0])
    fixBasePos = numpy.array([-13.125 * MMPerInch, -17.296 * MMPerInch, -193.0])
    fixedLink = mirrorCtrl.AdjLengthLink(fixBasePos, fixMirPos, -numpy.inf, numpy.inf, 31.495, 0.)
    actuatorList.append(fixedLink)
    encoderList.append(fixedLink)

    # minCorrList = [4.0e-5]*3 + [0]*2 # min correction (mm); 50 actuator microsteps
    # maxCorrList = [0.79]*3   + [0]*2 # max correction (mm); 1000000 actuator microsteps

    minCorrList = [50]*6 # min correction (microsteps)
    maxCorrList = [1000000]*3   + [1000000]*3 # max correction (microsteps)

    return mirrorCtrl.TipTransMirror(
        ctrMirZ = CtrMirZ,
        ctrBaseZ = CtrBaseZ,
        actuatorList = actuatorList,
        fixedLinkList = [],
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

mir25mSecOldModel = _makeMirror()
