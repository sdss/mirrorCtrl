#!/usr/bin/env python
"""This script contains configuration for the 2.5m Secondary mirror, and launches the galil actor.


To Do:
1. need to incorporate actual encoder positions
2. get actual address info
3. get mir/base positions for the anti-rotation link, it is currently being faked here.
"""
__all__ = ["DeviceInfo"]

import mirrorCtrl
import numpy

Name = '3.5m Secondary'

########### For testing ###############
GalilHost = 'localhost'
GalilPort = 8000 # matches fakeGalil.py for testing
#######################################

# choose the actuator model (adjustable base or adjustable length)
genLink = mirrorCtrl.AdjBaseActuator # new style
# genLink = mirrorCtrl.AdjLengthLink # old style

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

ActMirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
ActMirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
ActMirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
ActBaseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
ActBaseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
ActBaseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])

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


# Fake FixedLengthLink, need to determine actual mir/basePos.
linkLength = 150. #mm
mirRadius = 1000. #mm
fixMirPos = numpy.array([0., mirRadius, -152.806])
fixBasePos = numpy.array([linkLength, mirRadius, -152.806])
fixLinkList = []
fixLinkList.append(mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos))

encLinkList = None # need to get these too.

Mirror = mirrorCtrl.DirectMirror(actLinkList, fixLinkList, encLinkList, Name)

DeviceInfo = mirrorCtrl.GalilDeviceInfo(
    mirror = Mirror,
    host = GalilHost,
    port = GalilPort,
)
