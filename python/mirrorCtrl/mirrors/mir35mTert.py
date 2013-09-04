#!/usr/bin/env python
"""This script contains configuration for the 3.5m Tertiary mirror, and launches the galil actor.

The 3.5m tertiary has a different coordinate system convention:
+Z points towards secondary
+Y points towards instrument port
+X defined by right hand rule

The actuator placements are defined (below) in the plane of the mirror, so a
coord transform is necessary.  The coords and transform method are almost identical 
to the file: 3.5m M3 Actuator Positions 2008-04-18.py (in docs directory).  
Differences are:
1. Fake encoder positions are also computed here.
2. Fixed link geometry is from Nick Macdonald, the old version used 3
   infinitely long links fixed at different positions than they are in reality.
   This version uses short rods, attached at their true physical locations (although
   the X translation link wasn't in the solid model, so I guessed that it extends along
   +X, rather than -X).
   The old positions are included below for reference but commented out.
   
3.5m M3 Actuator Positions 2008-04-18.py has a worksheet showing old TCC 3.5m M3 model
   
Notes:

7/12    - Measured encoder / actuator positions during shutdown.
            actuator radius = 8.96"
            encoder radius = 10.69"
        - Fixed length links: 2 extend towards A (-y?), 1 extends towards B (+x?). 
            I think this is correct convention.
        - Did not adjust Z positions.
"""
__all__ = ["Mirror"]

import math
import numpy
import mirrorCtrl

Name = '3.5m Tertiary'

MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

#rad =   11.714 * MMPerInch # old
actRad =   8.96 * MMPerInch # updated 7/12 distance from center of actuator to center of mirror
encRad = 10.69 * MMPerInch # encoders are radially offset from actuators

zMir =  -0.875 * MMPerInch
zBase = -3.375 * MMPerInch
#angDegList = numpy.arange(-90.0, 359.0, 360.0 / 3.0) # bug?
angDegList = numpy.array([-90.0, 30., 150.])
angRadList = angDegList * RadPerDeg

# compute in-plane positions
# with convention:
# z points from back of glass to front of glass
# y points sort of towards the instrument
# x is as required for right-handed coordinate system
# Actuator index:
#   0-2 = axial actuators A-C
mirAct = numpy.zeros([3,3])
baseAct = numpy.zeros([3,3])
mirEnc = numpy.zeros([3,3])
baseEnc = numpy.zeros([3,3])
# first handle actuators A, B and C
# they are located at the angular positions in andRadList
for actInd, angRad in enumerate(angRadList):
    mirAct[actInd, :] = numpy.array((
        math.cos(angRad) * actRad,
        math.sin(angRad) * actRad,
        zMir
    ))
    baseAct[actInd, 0:2] = mirAct[actInd, 0:2]
    baseAct[actInd, 2] = zBase

    mirEnc[actInd, :] = numpy.array((
        math.cos(angRad) * encRad,
        math.sin(angRad) * encRad,
        zMir
    ))
    baseEnc[actInd, 0:2] = mirAct[actInd, 0:2]
    baseEnc[actInd, 2] = zBase
    
# now add fixed link constraints:
# 0,1 are transverse
# 2 is anti-rotation
# positions of fixed link ends (from Nick MacDonald drawing)
# measurements during 2012 3.5m shutdown are pretty damn close
mirFix = numpy.zeros([3, 3])
baseFix = numpy.zeros([3, 3])
mirFix[0, :] = numpy.array([-203.2, 0., 0.])
mirFix[1, :] = numpy.array([203.2, 0., 0.])
mirFix[2, :] = numpy.array([0., 0., 0.])
baseFix[0, :] = mirFix[0, :]#.copy()
baseFix[0, 1] = -281.47 # fixed links extend towards A
baseFix[1, :] = mirAct[1, :]#.copy()
baseFix[1, 1] = -281.47 # fixed links extend towards A
baseFix[2, :] = numpy.array([281.47, 0., 0.])



# rotate to final coordinate system which is:
# z points towards secondary
# y points towards the instrument port
# x is unchanged
# in other words, rotate 45 degrees about x
rotAng = -45.0 * math.pi / 180.0
rotMat = numpy.zeros([3,3])
rotMat[0,0] = 1
rotMat[1,1] = math.cos(rotAng)
rotMat[1,2] = math.sin(rotAng)
rotMat[2,2] = rotMat[1,1]
rotMat[2,1] = -rotMat[1,2]
mirPosAct = numpy.dot(mirAct, rotMat)
basePosAct = numpy.dot(baseAct, rotMat)
mirPosEnc = numpy.dot(mirEnc, rotMat)
basePosEnc = numpy.dot(baseEnc, rotMat)
mirPosFix = numpy.dot(mirFix, rotMat)
basePosFix = numpy.dot(baseFix, rotMat)

# Limits of motion (mount units) for each actuator
ActMinMount = numpy.array([-7250000., -7250000., -7250000])
ActMaxMount = numpy.array([ 7250000.,  7250000.,  7250000])
# Offset (mount units) and scale (mount units/um) for each actuator
ActMountOffset = numpy.array([      0.,       0.,       0.])
ActMountScale  = numpy.array([1259.843, 1259.843, 1259.843])

actuatorList = []
encoderList = []
fixedLinkList = []
# generate list of actuators, encoders, and fixed links
for i in range(3):    
    actuatorList.append(
        mirrorCtrl.AdjLengthLink(
        #mirrorCtrl.AdjBaseActuator(
            basePosAct[i, :], mirPosAct[i, :], ActMinMount[i], 
            ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
        )
    )    
    encoderList.append(
        mirrorCtrl.AdjLengthLink(
            basePosEnc[i, :], mirPosEnc[i, :], ActMinMount[i], 
            ActMaxMount[i], ActMountScale[i], ActMountOffset[i]        
        )
    )    
    fixedLinkList.append(
        mirrorCtrl.FixedLengthLink( basePosFix[i, :], mirPosFix[i, :] )
    )

# minCorrList = [4.0e-5]*3 # min correction (mm); 50 actuator microsteps
# maxCorrList = [0.79]*3   # max correction (mm); 1000000 actuator microsteps
    
minCorrList = [50]*3 # min correction actuator microsteps
maxCorrList = [1000000]*3   # max correction actuator microsteps
    
Mirror = mirrorCtrl.DirectMirror(
    actuatorList = actuatorList,
    fixedLinkList = fixedLinkList,
    encoderList = encoderList,
    minCorrList = minCorrList,
    maxCorrList = maxCorrList,
    name = Name,
)
