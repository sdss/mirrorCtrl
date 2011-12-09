#!/usr/bin/env python
"""This script generates fail case examples for select mirror-orientation combos
"""
import unittest
import numpy
import math

from data import genMirData
import mirror

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

# this is the max acceptable error for a round-tripped orientation
maxOrientErr = numpy.array([.1 * MMPerMicron, .01 * RadPerArcSec, \
.01 * RadPerArcSec, .1 * MMPerMicron, .1 * MMPerMicron, 1*RadPerDeg])

# these are the extremes that we expect to handle
maxDist = 25. #mm (translations)
maxTilt = 2. * RadPerDeg # 2 degrees (tilts)

def roundTrip(mir, orientIn):
    mnt, adjOrient = mir.actuatorMountFromOrient(orientIn, return_adjOrient=True)
    # adjOrient is adjusted Orient due to fixed links
    mnt = numpy.asarray(mnt, dtype=float)
    adjOrient = numpy.asarray(adjOrient, dtype=float)
    
    
    noiseAmp = numpy.asarray([maxDist, maxTilt, maxTilt, 
                                maxDist, maxDist, 0.], dtype=float) / 10. # no rotz noise
                                
    # noise is: (flat distribution between -1 and 1) * noiseAmp + adjOrient                            
    noisyOrient = (2 * numpy.random.random_sample(6,) - 1) * noiseAmp + adjOrient
    
    orient = mir.orientFromActuatorMount(mnt, noisyOrient)
    orient = numpy.asarray(orient, dtype=float)
    
    mnt2 = mir.actuatorMountFromOrient(orient)
    mnt2 = numpy.asarray(mnt2, dtype=float)
    orientErr = abs(adjOrient-orient)
    mountErr = abs(mnt - mnt2)
    
    if True in (orientErr > maxOrientErr):
        print 'Failed round trip orient:'
        print 'orientIn: ', orientIn
        print 'orientErr: ', orientErr
        print 'mount: ', mnt
        print 'mountErr: ', mountErr
        
    else: 
        print 'Passed round trip orient!'
    print
    print
    return
    
################################# CASE 1 #####################################
"""This is a version of the 2.5m M1 mirror with one of the transverse actuators held
constand (a fixed link).  This fails for almost all orientations.

The 3 transverse actuators attach to the mirror at roughly (x,y):
1. [0, +mir_radius] (extending in +X)
2. [0, -mir_radius] (extending in +X) <----THIS ONE IS FIXED!
3. [-mir_radius, 0] (extending in -Y)

"""
print '-------------------- CASE 1--------------------------'

# get actuator/encoder lists from genMirData and create a mirror
links = genMirData.prim25List[4]
# fyi for the version with the other actuator fixed: genMirData.prim25List[2]

mir = mirror.DirectMirror(*links)

# define a desired orientation
orientIn = [0., 0., 0., 25., 0.]

# round trip it

roundTrip(mir, orientIn)
    
