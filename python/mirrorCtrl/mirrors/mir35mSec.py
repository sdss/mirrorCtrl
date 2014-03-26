#!/usr/bin/env python2
"""Configuration of secondary mirror for 3.5m APO telescope

notes: 
2012-08 - Measured encoder and fixed link positions during 2012 shutdown.
            actuator radius = 9.08 inches
            encoder radius = 10.25 inches
            
        - do the transverse encoders lie between the actuator and glass?  I can't remember but I think this is correct
"""
__all__ = ["mir35mSec"]

import math
import numpy
import mirrorCtrl
from mirrorCtrl.const import MMPerInch, RadPerDeg

## Mirror name
Name = '3.5m Secondary'
## Radial position of ABC actuators relative to mirror vertex
actRad =   9.08 * MMPerInch 
## Radial position of ABC encoders relative to mirror vertex
encRad = 10.25 * MMPerInch 

def _makeMirror():
    """Create a 3.5m Secondary Mirror
    """
    #########################################################################
    ##################### old TCC config file, for reference ################
    """mir position x coordinate for each axis"""
    #ActMirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
    """mir position y coordinate for each axis"""
    #ActMirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
    """mir position z coordinate for each axis"""
    #ActMirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
    """base position x coordinate for each axis"""
    #ActBaseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
    """base position y coordinate for each axis"""
    #ActBaseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
    """base position z coordinate for each axis"""
    #ActBaseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])
    ###########################################################################

    zMirAx =  -152.806 # for Axial
    zBaseAx = -256.438 # for Axial
    zMirTrans = -167.361 # for Transverse
    zBaseTrans = -192.710 # for Transverse
    xyMirTrans = 29.186 # for Transverse
    xyBaseTrans = 284.010 # for Transverse
    zEncOffsetTrans = 0.90 * MMPerInch # Transverse encoders are between actuator and glass.

    
    angDegList = numpy.array([-90.0+180., 30.+180., 150.+180.]) 
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
        baseEnc[actInd, 0:2] = mirEnc[actInd, 0:2]
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
    actuatorList = []
    encoderList = []
    for i in range(5):
        ############## from mir.dat ################
        # basePosAct = [ActBaseX[i], ActBaseY[i], ActBaseZ[i]]
        # mirPosAct = [ActMirX[i], ActMirY[i], ActMirZ[i]]
        # basePosEnc = basePosAct[:]
        # mirPosEnc = mirPosAct[:]
        ###########################################
        ##### actual measurement ##################
        basePosAct = baseAct[i, :]
        mirPosAct = mirAct[i, :]
        basePosEnc = baseEnc[i, :]
        mirPosEnc = mirEnc[i, :]   
        ###########################################


        actuatorList.append(
            mirrorCtrl.AdjBaseActuator(
                basePosAct, mirPosAct, 
                ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]
            )
        )
        encoderList.append(
            mirrorCtrl.AdjLengthLink(
                basePosEnc, mirPosEnc, 
                ActMinMount[i], ActMaxMount[i], ActMountScale[i], ActMountOffset[i]        
            )
        )


    # FixedLengthLink
    # located between C and B on edge of mirror opposite of A.
    linkLength = 12.36 * MMPerInch # measured
    mirRadius = 1000. # guess
    fixMirPos = numpy.array([0., -1*mirRadius, zMirAx]) # opposite of A
    fixBasePos = numpy.array([linkLength, -1*mirRadius, zMirAx])
    fixedLinkList = [mirrorCtrl.FixedLengthLink(fixBasePos, fixMirPos)]

    # minCorrList = [4.0e-5]*3 + [0.0016]*2 # min correction (mm); 50 actuator microsteps for all actuators
    # maxCorrList = [0.79]*3   + [0.16]*2 # max correction (mm); 1000000 actuator microsteps for A-C; 5000 for D-E

    minCorrList = [50]*5 # min correction (microsteps)
    maxCorrList = [1000000]*3   + [5000]*2 # max correction (microsteps)


    return mirrorCtrl.DirectMirror(
        actuatorList = actuatorList,
        fixedLinkList = fixedLinkList,
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

mir35mSec = _makeMirror()
