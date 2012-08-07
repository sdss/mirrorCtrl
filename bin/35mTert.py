#!/usr/bin/env python
"""This script contains configuration for the 2.5m Primary mirror, and launches the galil actor.

The 3.5m tertiary has a different coordinate system convention:
+Z points towards secondary
+Y points towards instrument port
+X defined by right hand rule

The actuator placements are defined (below) in the plane of the mirror, so a
coord transform is necessary.  The coords and transform method are almost identical 
to the file: 3.5m M3 Actuator Positions 2008-04-18.py (in docs directory).  
Differences are:
1. Encoder positions are also computed here.
2. Fixed link geometry is from Nick Macdonald, the old version used 3
   infinitely long links fixed at different positions than they are in reality.
   This version uses short rods, attached at their true physical locations (although
   the X translation link wasn't in the solid model, so I guessed that it extends along
   +X, rather than -X).
   The old positions are included below for reference but commented out.
   
Notes:

7/12    - Measured encoder / actuator positions during shutdown.
            actuator radius = 8.96"
            encoder radius = 10.69"
        - Fixed length links: 2 extend towards A (-y?), 1 extends towards B (+x?). 
            I think this is correct convention.
        
         
"""
import math
import numpy
import mirrorCtrl

Name = '3.5m Tertiary'

UserPort = 3533   
########### For testing ###############
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing
#######################################

# choose the actuator model (adjustable base or adjustable length)
genLink = mirrorCtrl.AdjBaseActuator # new style, actuator
# genLink = mirrorCtrl.AdjLengthLink # old style
encLink = mirrorCtrl.AdjLengthActuator # encoder

# -------BEGIN copying from 3.5m M3 Actuator Positions 2008-04-18.py--------
MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

#rad =   11.714 * MMPerInch # old
actRad =   8.96 * MMPerInch # updated 7/12 distance from center of actuator to center of mirror
encRad = 10.69 * MMPerInch # encoders are radially offset from actuators

zMir =  -0.875 * MMPerInch
zBase = -3.375 * MMPerInch
#angDegList = numpy.arange(-90.0, 359.0, 360.0 / 3.0) # bug?
andDegList = numpy.array([-90.0, 30., 150.])
angRadList = angDegList * RadPerDeg

# compute in-plane positions
# with convention:
# z points from back of glass to front of glass
# y points sort of towards the instrument
# x is as required for right-handed coordinate system
# Actuator index:
#   0-2 = axial actuators A-C
#   3-5 = constraints
mirIP = numpy.zeros([6,3])
baseIP = numpy.zeros([6,3])
mirEnc = numpy.zeros([6,3])
baseEnc = numpy.zeros([6,3])
# first handle actuators A, B and C
# they are located at the angular positions in andRadList
for actInd, angRad in enumerate(angRadList):
    mirIP[actInd, :] = numpy.array((
        math.cos(angRad) * actRad,
        math.sin(angRad) * actRad,
        zMir
    ))
    baseIP[actInd, 0:2] = mirIP[actInd, 0:2]
    baseIP[actInd, 2] = zBase

    mirEnc[actInd, :] = numpy.array((
        math.cos(angRad) * encRad,
        math.sin(angRad) * encRad,
        zMir
    ))
    baseEnc[actInd, 0:2] = mirIP[actInd, 0:2]
    baseEnc[actInd, 2] = zBase
    
# now add constraints:
# 3,4 are transverse
# 5 is anti-rotation

# ------- old implementation ----------
# infinite links below are taken to be 1e4 to avoid
# roundoff error in new implementation (old version used 1e9)
# mirIP[3] = numpy.zeros(3)
# mirIP[4] = mirIP[3].copy()
# mirIP[5] = mirIP[3].copy()
# mirIP[5,0] = rad
# baseIP[3] = mirIP[3].copy()
# baseIP[3, 0:2] = (1.0e4, 1.0e4)   
# baseIP[4, 0:2] = (-1.0e4, 1.0e4)
# baseIP[5] = (rad, 1.0e4, 0)

# --------- drawing implementation ------------
# positions (from Nick MacDonald drawing)
# measurements during 2012 3.5m shutdown are very close
mirIP[3] = (-203.2, 0., 0.)
mirIP[4] = (203.2, 0., 0.)
mirIP[5] = (0., 0., 0.)
baseIP[3] = mirIP[3].copy()
baseIP[3, 1] = -(281.47) # fixed links extend towards A
baseIP[4] = mirIP[4].copy()
baseIP[4, 1] = -(281.47) # fixed links extend towards A
baseIP[5] = (281.47, 0., 0.)



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
mirPos = numpy.dot(mirIP, rotMat)
basePos = numpy.dot(baseIP, rotMat)
mirPosEnc = numpy.dot(mirEnc, rotMat)
basePosEnc = numpy.dot(baseEnc, rotMat)

# -------END (mostly) copying from 3.5m M3 Actuator Positions 2008-04-18.py--------
# from mir_35.dat:
# Limits of motion (mount units) for each actuator
ActMinMount = numpy.array([-7250000., -7250000., -7250000])
ActMaxMount = numpy.array([ 7250000.,  7250000.,  7250000])
# Offset (mount units) and scale (mount units/um) for each actuator
ActMountOffset = numpy.array([      0.,       0.,       0.])
ActMountScale  = numpy.array([1259.843, 1259.843, 1259.843])

# actuator positions (1st 3 columns of mir/basePos)
ActBaseX = basePos[0:3,0]
ActBaseY = basePos[0:3,1]
ActBaseZ = basePos[0:3,2]
ActMirX = mirPos[0:3,0]
ActMirY = mirPos[0:3,1]
ActMirZ = mirPos[0:3,2]

# encodcer positions (1st 3 columns of mir/basePos)
EncBaseX = basePosEnc[0:3,0]
EncBaseY = basePosEnc[0:3,1]
EncBaseZ = basePosEnc[0:3,2]
EncMirX = mirPosEnc[0:3,0]
EncMirY = mirPosEnc[0:3,1]
EncMirZ = mirPosEnc[0:3,2]

actLinkList = []
encLinkList = []
# generate list of actuators
for i in range(3):
    actBasePos = numpy.array([ActBaseX[i], ActBaseY[i], ActBaseZ[i]])
    actMirPos = numpy.array([ActMirX[i], ActMirY[i], ActMirZ[i]])
    encBasePos = numpy.array([EncBaseX[i], EncBaseY[i], EncBaseZ[i]])
    encMirPos = numpy.array([EncMirX[i], EncMirY[i], EncMirZ[i]])
    actMin = ActMinMount[i]
    actMax = ActMaxMount[i]
    actScale = ActMountScale[i]
    actOffset = ActMountOffset[i]
    actLink = genLink(actBasePos, actMirPos, actMin, actMax, actScale, actOffset)
    actLinkList.append(actLink)
    encLink = encLink(encBasePos, encMirPos, actMin, actMax, actScale, actOffset)
    encLinkList.append(encLink)

# fixed link positions (last 3 columns of mir/basePos)
FixBaseX = basePos[3:,0]
FixBaseY = basePos[3:,1]
FixBaseZ = basePos[3:,2]
FixMirX = mirPos[3:,0]
FixMirY = mirPos[3:,1]
FixMirZ = mirPos[3:,2]

# generate list of fixed length links
fixLinkList = []
for i in range(3):
    fixBasePos = numpy.array([FixBaseX[i], FixBaseY[i], FixBaseZ[i]])
    fixMirPos = numpy.array([FixMirX[i], FixMirY[i], FixMirZ[i]])
    fixLinkList.append(mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos))
    

Mirror = mirrorCtrl.DirectMirror(actLinkList, fixLinkList, encLinkList, Name)

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice35M3(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)


