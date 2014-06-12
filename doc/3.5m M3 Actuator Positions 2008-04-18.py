#!/usr/bin/env python2
import numpy
import math
"""3.5m M3 mirror actuator geometry

All references are as seen looking at the mirror from the instrument port,
with "up" being towards the secondary mirror.

Positions from Larry Carey were provided in the following "in-plane" coordinate system,
with X and Y in the plane of the mirror:
+X is to the right
+Z is from back of mirror to front of mirror
+Y is as required for right-handed: up and back

The actuators and encoders are lettered in a counter-clockwise direction, with A down.
(This is a 180 degree rotation from the positioning of the old tertiary actuators.)

The Heidenhains are on a radius of 11.714 in. from the origin, 120 deg. apart,
azmuthally aligned with the axial actuators. (The actuators are on a radius of 8.963")

Using the above convention, the nominal positions of the "flex points" for the Haidenhains are as follows, in 
inches (warning: the TCC uses mm):

Encoder    X         Y         Z Mir      Z Base
A          0       -11.714    -0.875     -3.375
B         10.145     5.857    -0.875     -3.375
C        -10.145     5.857    -0.875     -3.375

(Note: the distance from the back of the back face to the vertex of the mirror is approximately 4.97",
but the Heidenhain encoders are attached to the back of the front plate.)

The coordinate system used by the TCC as follows:
+Z points towards the secondary mirror
+Y points towards the instrument port
+X is as required for a right-handed coordinate system
   (this is to the right as seen by the instrument if Z is up)
which is the in-plane coordinate system rotated -45 degrees about the X axis.

So the positions are first computed "in plane", then rotated to give the values used by the TCC.
"""

MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

def printTable(name, pos):
    """Print a table of actuator positions.

    Inputs:
    - name: one of "Mir" or "Base"
    - pos: a matrix [actInd, coordInd]
    """
    for coordInd, coordName in enumerate(("X", "Y", "Z")):
        infoTuple = (name, coordName,) + tuple(pos[:,coordInd])
        print "TertAct%s%s %10.3f %10.3f %10.3f %10.3g %10.3g %10.3g" % infoTuple
    
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
baseIP = mirIP.copy()
# first handle actuators A, B and C
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
mirIP[3] = numpy.zeros(3)
mirIP[4] = mirIP[3].copy()
mirIP[5] = mirIP[3].copy()
mirIP[5,0] = rad
baseIP[3] = mirIP[3].copy()
baseIP[3, 0:2] = (1.0e9, 1.0e9)
baseIP[4, 0:2] = (-1.0e9, 1.0e9)
baseIP[5] = (rad, 1.0e9, 0)

print "Sanity check; unrotated positions in inches"
printTable("UMirInch", mirIP / MMPerInch)
print
printTable("UBaseInch", baseIP / MMPerInch)
print

# rotate to final coordinate system which is:
# z points towards secondary
# y points towards the instrument port
# x is unchanged
# in other words, rotate 45 degrees about x
rotAngRad = -45.0 * RadPerDeg
rotMat = numpy.zeros([3,3])
rotMat[0,0] = 1
rotMat[1,1] = math.cos(rotAngRad)
rotMat[1,2] = math.sin(rotAngRad)
rotMat[2,2] = rotMat[1,1]
rotMat[2,1] = -rotMat[1,2]

mirPos = numpy.dot(mirIP, rotMat)
basePos = numpy.dot(baseIP, rotMat)

print "Data for tdat:mir.dat"
printTable("Mir", mirPos)
printTable("Base", basePos)


