#!/usr/bin/env python
"""This script contains configuration for the 2.5m Secondary mirror, and launches the galil actor.

To Do: 
1. need to incorporate actual encoder positions
2. get actual address info
3. French's model places the anti-rotation arm Z height = 1 inch below the glass. I don't know
    what the z offset from the vertex of the mirror is for this dimension. So I adoped the Z
    position of the A,B,C actuators instead.  Also the X and Y axes were not labeled, so
    I am making an educated assumption about which is which. The drawings are currently in 
    the docs directory.

"""
import numpy
import mirror

Name = 'SDSS Secondary'

UserPort = 2532   
########### For testing ###############
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing
#######################################

# choose the actuator model (adjustable base or adjustable length)
genLink = mirror.AdjBaseActuator # new style
# genLink = mirror.AdjLengthLink # old style

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
actLinkList = []
for i in range(5):
    actBasePos = numpy.array([ActBaseX[i], ActBaseY[i], ActBaseZ[i]])
    actMirPos = numpy.array([ActMirX[i], ActMirY[i], ActMirZ[i]])
    actMin = ActMinMount[i]
    actMax = ActMaxMount[i]
    actScale = ActMountScale[i]
    actOffset = ActMountOffset[i]
    actLink = genLink(actBasePos, actMirPos, actMin, actMax, actScale, actOffset)
    actLinkList.append(actLink)

# Fixed Link from French's anti-rotation arm drawings.
# note: x and y poitions were not labeled, so may be transposed
MMPerInch = 25.4
# note: French's model has Z = 1 inch below glass, but I'm adopting Z = actuators' mir pos.
fixLinkList = [] 
fixMirPos = numpy.array([0., -17.296 * MMPerInch, -193.0])
fixBasePos = numpy.array([13.125 * MMPerInch, -17.296 * MMPerInch, -193.0])
fixLinkList.append(mirror.FixedLengthLink(fixBasePos, fixMirPos))

encLinkList = None # need to get these positions from French.

Mirror = mirror.TipTransMirror(CtrMirZ, CtrBaseZ, actLinkList, fixLinkList, encLinkList, Name)

if __name__ == "__main__":
    device = mirror.GalilDevice25M2(
        mirror = Mirror,
        host = GalilHost,
        port = GalilPort,
    )
    mirror.runMirrorController(device = device, userPort = UserPort)
