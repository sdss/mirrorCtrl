#!/usr/bin/env python
"""This script contains configuration for the 2.5m Secondary mirror, and launches the galil actor.

notes: 
    8/12    - Measured encoder and fixed link positions during 2012 shutdown.
            actuator radius = 9.08 inches
            encoder radius = 10.25 inches
            
            - do the transverse encoders lie between the actuator and glass?  I can't remember
                but I think this is correct
"""
__all__ = ["Mirror"]

import math
import numpy
import mirrorCtrl

Name = '3.5m Secondary'

# old TCC config file, for reference
# ActMirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
# ActMirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
# ActMirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
# ActBaseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
# ActBaseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
# ActBaseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])

MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

actRad =   9.08 * MMPerInch # updated 7/12 distance from center of actuator to center of mirror
encRad = 10.25 * MMPerInch # encoders are radially offset from actuators

zMirAx =  -152.806 # for Axial
zBaseAx = -256.438 # for Axial
zMirTrans = -167.361 # for Transverse
zBaseTrans = -192.710 # for Transverse
xyMirTrans = 29.186 # for Transverse
xyBaseTrans = 284.010 # for Transverse
zEncOffsetTrans = 0.90 * MMPerInch # Transverse encoders are between actuator and glass.

angDegList = numpy.array([-90.0, 30., 150.])
angRadList = angDegList * RadPerDeg

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
# first handle axial actuators A, B and C
# they are located at the angular positions in angRadList
for actInd, angRad in enumerate(angRadList):
    mirAct[actInd, :] = numpy.array([
        math.cos(angRad) * actRad,
        math.sin(angRad) * actRad,
        zMirAx
    ])
    baseAct[actInd, 0:2] = mirAct[actInd, 0:2]
    baseAct[actInd, 2] = zBaseAx

    mirEnc[actInd, :] = numpy.array([
        math.cos(angRad) * encRad,
        math.sin(angRad) * encRad,
        zMirAx
    ])
    baseEnc[actInd, 0:2] = mirAct[actInd, 0:2]
    baseEnc[actInd, 2] = zBaseAx

# now handle Transverse actuator / encoders D, E.
mult = 1.
for actInd in range(3, 5):
    # actuators
    mirAct[actInd, :] = numpy.array([
        xyMirTrans * mult,
        xyMirTrans,
        zMirTrans
    ])
    baseAct[actInd, :] = numpy.array([
        xyBaseTrans * mult,
        xyBaseTrans,
        zBaseTrans
    ])
    
    mirEnc[actInd, :] = numpy.array([
        xyMirTrans * mult,
        xyMirTrans,
        zMirTrans + zEncOffsetTrans
    ])
    baseEnc[actInd, :] = numpy.array([
        xyBaseTrans * mult,
        xyBaseTrans,
        zBaseTrans + zEncOffsetTrans
    ])
    mult = -1.

# generate lists of link objects for mirror configuration
actLinkList = []
encLinkList = []
for i in range(5):    
    actLinkList.append(
        mirrorCtrl.AdjBaseActuator(
            baseAct[i, :], mirAct[i, :], 
            ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
        )
    )
    encLinkList.append(
        mirrorCtrl.AdjLengthLink(
            baseEnc[i, :], mirEnc[i, :], 
            ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]        
        )
    )


# FixedLengthLink
# located between C and B on edge of mirror opposite of A.
linkLength = 12.36 * MMPerInch # measured
mirRadius = 1000. # guess
fixMirPos = numpy.array([0., mirRadius, zMirAx]) # opposite of A
fixBasePos = numpy.array([linkLength, mirRadius, zMirAx])
fixLinkList = [mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos)]

Mirror = mirrorCtrl.DirectMirror(actLinkList, fixLinkList, encLinkList, Name)
