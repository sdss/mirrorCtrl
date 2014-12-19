"""Configuration of primary mirror for 2.5m SDSS telescope

notes: may wish to:
    increase mount scale?
    get actual address info
    use actual length links (not inf)?
    attach trans y to mirror vertex?
"""
__all__ = ["mir25mPrimFortranTest"]

import numpy

import mirrorCtrl

Name = 'mir25mPrimFortranTest'

## choose the actuator model (adjustable base or adjustable length)
genLink = mirrorCtrl.AdjLengthLink # old style
#genLink = mirrorCtrl.AdjBaseActuator # new style

def _makeMirror():
    """Construct a 2.5m Primary Mirror

    Info taken from mir.dat file
    """
    # Warnings:
    # - most of this info is from drawings and may be a bit off
    # - M1 offsets are based on mechanical centering of the mirror
    #   the actuator lengths may be somewhat incorrect
    # - M2 offsets are based on feeler gauge measurements of
    #   the position of the actuator puck in the hole
    #-
    #+
    #      Primary Mirror
    #
    # The primary mirror is positioned by three axial actuators and one transverse
    # actuator (along x, acting up/down when the mirror is at horizon) that control
    # hard points for an air support servo system. In addition, the x translation
    # of the mirror is controlled by two lateral links, one at the top of the mirror
    # and the other at the bottom.
    #
    # After a y translation the mirror will have moved, but it will have slid
    # across high friction air pistons. Relaxing the mirror support system
    # by bringing the telescope near the zenith and then repeatedly lowering
    # the mirror off air and then raising it again.
    #
    # The lateral links are not strong enough to move the mirror, so after an x
    # translation one must walk the mirror over by relaxing it as described above.
    #-


    # Actuators are: Axial A, B, C, Trans Vert, Lower Lateral, Upper Lateral
    # Limits of motion (mount units) for each actuator
    ActMinMount = numpy.array([-120000., -120000., -120000., -90000., -50000., -50000])
    ActMaxMount = numpy.array([ 120000.,  120000.,  120000.,  90000.,  50000.,  50000])
    # Axial A, B, C and Trans Vert are 40 tpi, 50 microsteps/step, 200 steps/rev
    # Lateral Links are 80 tpi, 50 microsteps/step, 200 steps/rev
    # Offset (mount units) and scale (mount units/um) for each actuator
    # This PrimMountOffset value is for M1 to be collimated on the sky with PrimX/YTiltCoef = 0
    # Russell Owen put these in 2011-10-04.
    # Before that change the tilt coefficients in tinst:default.dat were:
    # PrimXTiltCoef      -24.31       0.0       0.0
    # PrimYTiltCoef        11.3       0.0       0.0
    # and the corresponding PrimMountOffset in this file (values to physically center
    # the primary mirror in its cell) were:
    # PrimMountOffset  9700       800       5650      -1650      -6900      -6900
    ActMountOffset = numpy.array([11300.,  -650.,  5500., -1650., -6900., -6900.])
    # do we want a higher resolution scale?
    ActMountScale = numpy.array([15.696, 15.696, 15.696, 15.696,  33.22, 32.53])

    # Axial A, B, C and Trans Vert all are not attached to the primary, so the actuator
    # does not bend over when the mirror translates perpendicularly to them.
    # The closest I think I can get to modelling this with the current TCC software
    # is to specify a very large length for the actuator. It's not perfect, but then
    # neither is any system that allows the actuator to contact the mirror wherever
    # it feels like.
    # Actual base positions for these actuators are as approximately as follows
    # Axial A, B, C base Z = 340.00
    # Trans Vert base Y = -1394.00
    ActMirX  = numpy.array([    0., -749.03,  749.03,     0.,     0.,    0.])
    ActMirY  = numpy.array([864.90, -432.45, -432.45, -1305., -1277., 1277.])
    # attach trans y act to vertex, uncomment below
    #MirY  = numpy.array([864.90, -432.45, -432.45,      0., -1277., 1277.])
    ActMirZ  = numpy.array([  251.,    251.,    251.,   238.,   262.,  262.])

    # A more finite version (replaced 9e9 with 9e4):
    ActBaseX = numpy.array([    0., -749.03,  749.03,     0.,  -698., -698.])
    ActBaseY = numpy.array([864.90, -432.45, -432.45,   -9e9, -1277., 1277.])
    ActBaseZ = numpy.array([   9e9,     9e9,     9e9,   238.,   262.,  262.])

    # generate a lists of link objects for mirror configuration
    fixedLinkList = [] # no fixed length links on SDSS primary
    actuatorList = []
    for i in range(6):
        actBasePos = numpy.array([ActBaseX[i], ActBaseY[i], ActBaseZ[i]])
        actMirPos = numpy.array([ActMirX[i], ActMirY[i], ActMirZ[i]])
        actMin = ActMinMount[i]
        actMax = ActMaxMount[i]
        actScale = ActMountScale[i]
        actOffset = ActMountOffset[i]
        actLink = genLink(actBasePos, actMirPos, actMin, actMax, actScale, actOffset)
        actuatorList.append(actLink)

    encoderList = None # no encoders on SDSS primary
    minCorrList = None
    maxCorrList = None

    return mirrorCtrl.DirectMirror(
        actuatorList = actuatorList,
        fixedLinkList = fixedLinkList,
        encoderList = encoderList,
        minCorrList = minCorrList,
        maxCorrList = maxCorrList,
        name = Name,
    )

mir25mPrimFortranTest = _makeMirror()
