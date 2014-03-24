#!/usr/bin/env python2
import numpy
import math
"""3.5m M3 mirror actuator worksheet

Compute data for the Galil file "Constants 35m M2.gal" and the TCC file tdat:mir.dat

The TCC code assumes that a flexure is attached at one end to the base
and the other end to the mirror, and that the actuator makes the flexure longer or shorter
Thus this worksheet computes the position of the flex point at each end of the flexure
when the mirror is in its neutral position (centered).
However, the 3.5m actuators actually work by having a linear actuator
(which is attached to the base) move the "base" end of the flexure along a line.
This the TCC's model is only an approximation.

Basics:
- all distances are in mm and all angles are in degrees unless otherwise noted
- all numbers are for the 3.5m secondary mirror support system
- X = right, Y = up, Z = from the sky towards the telescope
where up/down/left/right are as seen with the telescope at the horizon,
  standing behind the primary mirror, looking towards the secondary mirror.
"""
MMPerInch = 25.4
RadPerDeg = math.pi / 180.0

def printTCCLine(name, axVal, trVal, fmtStr):
    axStr = fmtStr % axVal
    trStr = fmtStr % trVal
    print name, " ".join([axStr]*3), " ".join([trStr]*2)
    
def printTCCPosTable(name, pos):
    """Print a table of actuator positions.

    Inputs:
    - name: one of "Mir" or "Base"
    - pos: a matrix [actInd, coordInd]
    """
    for coordInd, coordName in enumerate(("X", "Y", "Z")):
        infoTuple = (name, coordName,) + tuple(pos[:,coordInd])
        print "SecAct%s%s %10.3f %10.3f %10.3f %10.3f %10.3f" % infoTuple

def getMirGeometry():
    """Return mirror geometry for mirror attachment and base attachment points
    
    Returns:
    - mirPos[actInd, coordInd]
    - basePos[actInd, coordInd]
    where actInd is:
    - 0: Axial A
    - 1: Axial B
    - 2: Axial C
    - 3: Transverse D
    - 4: Transverse E
    (the rotational constraint is implied)
    """
    
    """Axial actuator geometry
    
    There are three axial actuators perpendicular to the mirror
    and spaced evenly at 120 degree angles
    """
    axRad =    10.480 * MMPerInch   # radius from axMirror center to axial encoders
    axMirZ =   -6.016 * MMPerInch   # z position of axial actuator mirror attachment
    axBaseZ = -10.096 * MMPerInch   # z position of axial actuator base attachment
    axAngA = 90.0   # angle from X to Y axis of axial actuator A
    angDegList = numpy.arange(axAngA, 359.0, 360.0 / 3.0)
    angRadList = angDegList * RadPerDeg
    
    mirPos = numpy.zeros([5,3])
    basePos = mirPos.copy()
    for actInd, angRad in enumerate(angRadList):
        mirPos[actInd, :] = numpy.array((
            math.cos(angRad) * axRad,
            math.sin(angRad) * axRad,
            axMirZ
        ))
        basePos[actInd, 0:2] = mirPos[actInd, 0:2]
        basePos[actInd, 2] = axBaseZ
    
    """Transverse Actuator geometry
    
    Transverse motion is controlled by two actuators that attach
    at the mirror end near the center of the back of the mirror
    and at the edge of the cage are pulled by Eastern Air Devices
    linear actuators (motors that turn hollow nuts)
    The encoders sense the extension of the screw that goes into the linear actuator
    I approximate this by assuming the flexure gets longer or shorter
    and that the encoder directly measures the length of the flexure
    1: Transverse D is upper right
    2: Transverse E is upper left
    """
    trMirZ   = -6.589 * 25.4    # z position of mirror end of transverse actuator flexure
    trBaseZ  = -(7.337 + 7.837) * 25.4 / 2  # z position of base end of transverse actuator
    trMirRad =  1.625 * 25.4    # radial distance from center of mirror to mirror end of transverse actuator flexure
    trBaseRad = 15.813 * 25.4   # radial distance from center of mirror to base end of transverse actuator
    trAng = numpy.array((45.0, 135.0))
    
    for actInd, trAngDeg in enumerate((45.0, 135.0)):
        trInd = actInd + 3
        trAngRad = trAngDeg * RadPerDeg
        mirPos[trInd, :] = (
            trMirRad * math.cos(trAngRad),
            trMirRad * math.sin(trAngRad),
            trMirZ
        )
        basePos[trInd, :] = (
            trBaseRad * math.cos(trAngRad),
            trBaseRad * math.sin(trAngRad),
            trBaseZ
        )
    return mirPos, basePos

"""
Actuator resolution and range of motion
"""

# Actuator scale (resolution)
# Axial: 40 thread/inch * 80:1 harmonic drive reduction * 200 full steps/rev * 50 microstep/step
axScaleMicrostepPerInch = 40 * 80 * 200 * 50 
axScaleMicrostepPerMicrometer = axScaleMicrostepPerInch / (25.4 * 1000)

# Transverse: -- 80 thread/inch * 200 full steps/rev * 50 microstep/full step
trScaleMicrostepPerInch = 80 * 200 * 50 
trScaleMicrostepPerMicrometer = trScaleMicrostepPerInch / (25.4 * 1000)

# Encoder resolution
# Axial: 2 um period, 200x interpolation
axEncUMPerTick = 2 * 200
axEncMicrostepPerTick = axScaleMicrostepPerMicrometer / axEncUMPerTick

# Transverse: 2 um period, 10x interpolation
trEncUMPerTick = 2 * 10
trEncMicrostepPerTick = trScaleMicrostepPerMicrometer / trEncUMPerTick

actScale = (axScaleMicrostepPerMicrometer,)*3 + (trScaleMicrostepPerMicrometer,) * 2

# Actuator range of motion
axFullRangeMicrostep = 15300000
trFullRangeMicrostep = 200000
axMarginInch = 0.0125
trMarginInch = 0.00625
axMarginMicrostep = int(0.5 + (axMarginInch * axScaleMicrostepPerInch))
trMarginMicrostep = int(0.5 + (trMarginInch * trScaleMicrostepPerInch))
axUsableRangeMicrostep = axFullRangeMicrostep - (2 * axMarginMicrostep)
trUsableRangeMicrostep = trFullRangeMicrostep - (2 * trMarginMicrostep)

print "\nData for \"Constants 35m M2.gal\""

print """
NO: distance between software limit and limit switch (microsteps)
MARGA = %d;  NO: %.4f inches
MARGB = MARGA
MARGC = MARGA
MARGD = %d;  NO: %.5f inches
MARGE = MARGD""" % (axMarginMicrostep, axMarginInch, trMarginMicrostep, trMarginInch)

print """
NO: full range of motion (microsteps)
RNGA = %d - (2 * MARGA)
RNGB = RNGA
RNGC = RNGA
RNGD = %d - (2 * MARGD)
RNGE = RNGD""" % (axFullRangeMicrostep, trFullRangeMicrostep)

print """
NO: resolution of encoder (microsteps/encoder tick), 0 if no encoder
NO: for a servo motor, this is the auxiliary encoder
ENCRESA = %.4f
ENCRESB = ENCRESA
ENCRESC = ENCRESA
ENCRESD = %.4f
ENCRESE = ENCRESD""" % (axEncMicrostepPerTick, trEncMicrostepPerTick)

mirPos, basePos = getMirGeometry()

print "\nData for TCC file tdat:mir.dat\n"
print "! Limits of motion (mount units) for each actuator"
printTCCLine("SecMinMount", -axUsableRangeMicrostep/2, -trUsableRangeMicrostep/2, "%10d")
printTCCLine("SecMaxMount",  axUsableRangeMicrostep/2,  trUsableRangeMicrostep/2, "%10d")
print "! Offset (mount units) and scale (mount units/um) for each actuator"
print "SecMountOffset     0          0          0          0          0"
printTCCLine("SecMountScale", axScaleMicrostepPerMicrometer, trScaleMicrostepPerMicrometer, "%10.3f")
print """! All distances are in mm, all angles are in degrees
! All numbers are for the 3.5m secondary mirror support system
! X = right, Y = up, Z = from the sky towards the telescope
! where up/down/left/right are as seen with the telescope at the horizon,
! standing behind the primary mirror, looking towards the secondary mirror.
!              Ax A       Ax B       Ax C       Tr D       Tr E"""
printTCCPosTable("Mir", mirPos)
printTCCPosTable("Base", basePos)
