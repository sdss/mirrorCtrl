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
1. Fake encoder positions are also computed here.
2. Fixed link geometry is from Nick Macdonald, the old version used 3
   infinitely long links fixed at different positions than they are in reality.
   This version uses short rods, attached at their true physical locations (although
   the X translation link wasn't in the solid model, so I guessed that it extends along
   +X, rather than -X).
   The old positions are included below for reference but commented out.
   
Notes:

Need to add encoder positions!
"""
import mirror
import numpy
import math

UserPort = ''
ControllerAddr = ''
ControllerPort = ''
########### For testing ###############
#UserPort = 1025   
#ControllerAddr = 'localhost'
#ControllerPort = 8000 # matches twistedGalil.py for testing
#######################################

# choose the actuator model (adjustable base or adjustable length)
genLink = mirror.AdjBaseActuator # new style
# genLink = mirror.AdjLengthLink # old style

# -------BEGIN copying from 3.5m M3 Actuator Positions 2008-04-18.py--------
MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

rad =   11.714 * MMPerInch
zMir =  -0.875 * MMPerInch
zBase = -3.375 * MMPerInch
angDegList = numpy.arange(-90.0, 359.0, 360.0 / 3.0)
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
# first handle actuators A, B and C
# they are located at the angular positions in andRadList
for actInd, angRad in enumerate(angRadList):
    mirIP[actInd, :] = numpy.array((
        math.cos(angRad) * rad,
        math.sin(angRad) * rad,
        zMir
    ))
    baseIP[actInd, 0:2] = mirIP[actInd, 0:2]
    baseIP[actInd, 2] = zBase
    
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

# --------- new implementation ------------
# actual positions (from Nick MacDonald)
mirIP[3] = (-203.2, 0., 0.)
mirIP[4] = (203.2, 0., 0.)
mirIP[5] = (0., 0., 0.)
baseIP[3] = mirIP[3].copy()
baseIP[3, 1] = (281.47)
baseIP[4] = mirIP[4].copy()
baseIP[4, 1] = (281.47) 
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

actLinkList = []
# generate list of actuators
for i in range(3):
    actBasePos = numpy.array([ActBaseX[i], ActBaseY[i], ActBaseZ[i]])
    actMirPos = numpy.array([ActMirX[i], ActMirY[i], ActMirZ[i]])
    actMin = ActMinMount[i]
    actMax = ActMaxMount[i]
    actScale = ActMountScale[i]
    actOffset = ActMountOffset[i]
    actLink = genLink(actBasePos, actMirPos, actMin, actMax, actScale, actOffset)
    actLinkList.append(actLink)

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
    fixLinkList.append(mirror.FixedLengthLink(fixBasePos, fixMirPos))
    
encLinkList = None # need to eventually obtain encoder positions!

# make the mirror
name = '3.5m Tertiary'
Mir = mirror.DirectMirror(actLinkList, fixLinkList, encLinkList, name)
Dev = mirror.GalilDevice35M3
# start up actor
if __name__ == "__main__":
    mirror.runGalil(Mir, Dev, UserPort, ControllerAddr, ControllerPort)

    


