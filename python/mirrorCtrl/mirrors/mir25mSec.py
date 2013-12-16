#!/usr/bin/env python
"""Configuration of the 2.5m Secondary mirror

To Do: 
1. need to incorporate actual encoder positions
2. get actual address info
3. French's model places the anti-rotation arm Z height = 1 inch below the glass. I don't know
    what the z offset from the vertex of the mirror is for this dimension. So I adoped the Z
    position of the A,B,C actuators instead.  Also the X and Y axes were not labeled, so
    I am making an educated assumption about which is which. The drawings are currently in 
    the docs directory.
    
notes:
    8/12    - was able to get rough measurment of encoder offset relative to
            axial actuators.  It is not radial.  They are at the same radius as
            actuators but about 2 inches away (a greater theta)
            Could not measure transverse encoder offset, will assume the 
            same offset as 3.5m secondary for now
"""
__all__ = ["Mirror"]

import numpy
import mirrorCtrl
from mirrorCtrl.const import MMPerInch

## Mirror Name
Name = 'SDSS Secondary'

def _makeMirror():
    """Construct a 2.5m Secondary mirror
    """

    # from 3.5m secondary.  THIS IS NOT VERIFIED TO BE TRUE FOR THIS MIRROR!!!!!
    zEncOffsetTrans = 0.90 * MMPerInch # Transverse encoders are between actuator and glass.

    # Warnings:
    # - most of this info is from drawings and may be a bit off
    # - M2 offsets are based on feeler gauge measurements of
    #   the position of the actuator puck in the hole
    #-
    #+
    #+
    #      Secondary Mirror
    #
    # The secondary mirror has three axial actuators. In addition,
    # transverse position is set by a linear bearing that is tilted
    # by two transverse actuators.


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
        actuatorList.append(
            mirrorCtrl.AdjBaseActuator(
                baseAct, mirAct, ActMinMount[i], 
                ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )
        # encoders lead actuators along same radius by approx 2 inches
        # I will estimate encoder position here based on previously accepted actuator positions
        if i in [0, 1, 2]:
            # axial encoder
            radius = numpy.sqrt(numpy.sum(mirAct[0:2]**2))
            theta = numpy.arctan2(mirAct[1], mirAct[0])
            # offset by 2 inch along circular arc
            deltaTheta = 2 * MMPerInch / radius
            newTheta = theta + deltaTheta
            xPos = radius * numpy.cos(newTheta)
            yPos = radius * numpy.sin(newTheta)
            encoderList.append(
                mirrorCtrl.AdjLengthLink(
                    numpy.array([xPos, yPos, baseAct[2]]),
                    numpy.array([xPos, yPos, mirAct[2]]),
                    ActMinMount[i], ActMaxMount[i], 
                    ActMountScale[i], ActMountOffset[i]
                )
            )
        else:
            # transverse encoders
            encoderList.append(
                mirrorCtrl.AdjLengthLink(
                    numpy.array([baseAct[0], baseAct[1], baseAct[2] + zEncOffsetTrans]),
                    numpy.array([mirAct[0], mirAct[1], mirAct[2] + zEncOffsetTrans]),
                    ActMinMount[i], ActMaxMount[i], 
                    ActMountScale[i], ActMountOffset[i]
                )
            )


    # Fixed Link from French's anti-rotation arm drawings.
    # note: x and y poitions were not labeled, so may be transposed

    # note: French's model has Z = 1 inch below glass, but I'm adopting Z = actuators' mir pos.
    fixedLinkList = [] 
    fixMirPos = numpy.array([0., -17.296 * MMPerInch, -193.0])
    fixBasePos = numpy.array([13.125 * MMPerInch, -17.296 * MMPerInch, -193.0])
    fixedLinkList.append(mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos))

    # minCorrList = [4.0e-5]*3 + [0]*2 # min correction (mm); 50 actuator microsteps
    # maxCorrList = [0.79]*3   + [0]*2 # max correction (mm); 1000000 actuator microsteps

    minCorrList = [50]*5 # min correction (microsteps)
    maxCorrList = [1000000]*3   + [1000000]*2 # max correction (microsteps)

    return mirrorCtrl.TipTransMirror(
        ctrMirZ = CtrMirZ,
        ctrBaseZ = CtrBaseZ,
        actuatorList = actuatorList,
        fixedLinkList = fixedLinkList,
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

## The 2.5m Secondary mirror construction
Mirror = _makeMirror()
