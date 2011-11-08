#!/usr/bin/env python
import math
import numpy
import warnings
import time
import matplotlib.pyplot
import link
import mirror

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

numpy.set_printoptions(precision=5, linewidth=132, suppress=True)

def printOrient(msgStr, orient):
    """Print orientation in user-friendly units"""
    print '%s: %0.2fum %0.3f" %0.3f" %0.2fum %0.2fum %0.3f"' % (
        msgStr,
        orient[0] / MMPerMicron,
        orient[1] / RadPerArcSec,
        orient[2] / RadPerArcSec,
        orient[3] / MMPerMicron,
        orient[4] / MMPerMicron,
        orient[5] / RadPerArcSec,
    )


reload(link)
reload(mirror)

# creates Link lists for all APO mirrors
# Actuator Lists:
# mir25_prim_actList -- direct
# mir25_sec_actList -- tip-trans, only 5 actuators defined
# mir35_sec_actList -- direct, only 5 actuators
# mir35_tert_actList -- direct with 3 fixed links
# Encoder Lists:
# mir25_prim_encList -- for testing only. Locations are imagined.
import parseMirFile
reload(parseMirFile)

# using old actuator geometry
actList = parseMirFile.mir25_prim_actList
encList = parseMirFile.mir25_prim_encList


#orientTest = mirror.Orientation(5, 0.02, 5e-4, 5, 5, 0)
orientTest = mirror.Orientation(0, 0, 0.02, 5, 0, 0)
print
print
print '-------------------------- New Run -------------------------'

printOrient("initial orient", orientTest)
print '-------------------------- no fixed link using encoders -------------------------'

dirMir = mirror.DirectMirror(actList, [], encList)
print "act phys at neutral: ", dirMir._physFromOrient([0]*6, dirMir.actuatorList)



mount1 = dirMir.actuatorMountFromOrient(orientTest)
print 'actuator mount1: ', numpy.array(mount1)
t1=time.time()
orient1 = dirMir.orientFromActuatorMount(mount1)
print 'time: ', time.time() - t1

orientErr = numpy.array(orient1) - orientTest
printOrient("orient error", orientErr)

mount2 = dirMir.actuatorMountFromOrient(orient1)
mountError = numpy.array(mount2) - mount1
print "actuator mount error: ", mountError

print
print '-------------------------- fixed link with encoder-------------------------'

printOrient('initial orient', orientTest)
flActList = actList[0:-1]
fixedLink = actList[-1]
enc2=encList[0:5]
fixedLinkList = [link.FixedLengthLink(basePos = fixedLink.basePos, mirPos = fixedLink.mirPos)]
Mir = mirror.DirectMirror(flActList, fixedLinkList, enc2)
mount2 = Mir.actuatorMountFromOrient(orientTest)
print 'mount2 : ', mount2
t1=time.time()
orient2 = Mir.orientFromActuatorMount(mount2)
print 'time: ', time.time() - t1

orientErr = numpy.array(orient2) - orientTest
printOrient("orient error", orientErr)

mount3 = Mir.actuatorMountFromOrient(orient2)
mountError = numpy.array(mount3) - mount2
print "actuator mount error: ", mountError

# print
# print '-------------------------- tip trans no fixed link with encoder-------------------------'
# printOrient('initial orient', orientTest)
# 
# actList = parseMirFile.mir25_sec_actList
# print len(actList)
# flActList = actList[0:-1]
# print len(flActList)
# fixedLink = actList[-1]
# 
# SecCtrMirZ = -135.70
# SecCtrBaseZ = -178.40
# 
# 
# dirMir = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, flActList, fixedLinkList, flActList)
# 
# print "act phys at neutral: ", dirMir._physFromOrient([0]*6, dirMir.actuatorList)
# 
# 
# 
# mount1 = dirMir.actuatorMountFromOrient(orientTest)
# print 'actuator mount1: ', numpy.array(mount1)
# t1=time.time()
# orient1 = dirMir.orientFromActuatorMount(mount1)
# print 'time: ', time.time() - t1
# 
# orientErr = numpy.array(orient1) - orientTest
# printOrient("orient error", orientErr)
# 
# mount2 = dirMir.actuatorMountFromOrient(orient1)
# mountError = numpy.array(mount2) - mount1
# print "actuator mount error: ", mountError
