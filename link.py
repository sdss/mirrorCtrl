"""Links

Mirrors are constrained by a set of 6 links: a mix of actuators and fixed-length links.

Links have two pivot points that are assumed to be perfect ball joints,
one at a fixed position on the mirror, the other at a fixed position
with respect to the mirror support structure.

There are three kinds of links supported in this file:
- FixedLengthLink: length does not change
- AdjLengthLink: length between base and mount changes. These are easy to model.
  They are the traditional hexapod actuators. The old TCC assumes all actuators
  are this kind, but we don't actually have any.
- AdjBaseLink: base pistons. This actuator introduces the concept of the home base position
  and the current base position. The secondary and tertiary mirrors use these.

Notes:
- The difference between AdjLengthLink and AdjBaseLink may be small enough to be ignored.
  AdjBaseLink is simpler.
- The 2.5m primary mirror has pistons that touch glass without being attached to it.
  Either AdjLengthLink or AdjBaseLink is probably a usable model for this.
- Links have a mirror position that is a ball joint attached somewhere to the mirror,
  or in some cases to the back of a shaft along which the mirror can piston.
- Links have a base position that is a ball joint fixed to a frame.

Actuators and Encoders:
- Actuators are links that control the length of the adjustable element
- Encoders are links that sense the length of the adjustable element
- Mirrors must have exactly 6 actuators or fixed-length links
- Mirrors also have 0 or 1 encoder for each actuator. The encoder does not have to be
  perfectly coaligned, but is assumed to be somewhat nearby.
"""
import numpy
import math

class BaseLink(object):
    """
    A base class for linear actuators and fixed links.
    Links are controlled in mount coordinates = steps, but all positions are in mm.
    """
    def __init__(self, isAdjustable, basePos, mirPos):
        """
        Inputs:
        - isAdjustable: True for actuators, False for fixed links.
        - basePos:      cartesian position of end of actuator fixed to the base (mm)
                        when the mirror is at zero orientation (for most links
                        this is always the base position).
        - mirPos:       cartesian position of end of actuator attached to the mirror (mm)
                        when the mirror is at zero orientation
        
        Zero orientation means piston, x tilt, y tilt, x translation, y translation and z rotation
        are all zero.
        """
        self.isAdjustable = bool(isAdjustable)
        self.basePos = numpy.asarray(basePos, dtype=float)
        if self.basePos.shape != (3,):
            raise RuntimeError("basePos=%s must be 3 elements" % (basePos,))
        self.mirPos = numpy.asarray(mirPos, dtype=float)
        if self.mirPos.shape != (3,):
            raise RuntimeError("mirPos=%s must be 3 elements" % (mirPos,))
        self.neutralLength = self.lengthFromMirPos(self.mirPos)
    
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        
        Subclasses must override.
        """
        raise NotImplementedError("Subclasses must define")

    def lengthFromMirPos(self, mirPos):
        """
        Return the desired length of link in mm.
        
        This is the distance between the supplied mirPos and the original basePos.
        The link may not be capable of achieving this length.
        """        
        return numpy.linalg.norm(self.basePos - mirPos)


class AdjLengthLink(BaseLink):
    """Adjustable-length actuator. The distance between the two ball joints varies.
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos:  cartesian position of end of actuator fixed to the base (mm)
        - mirPos:   cartesian position of end of actuator attached to the mirror (mm)
                    when the mirror is at zero orientation
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale:    actuator scale (mm/step)
        - offset:   mount position when mirror is at zero orientation
        """
        BaseLink.__init__(self, isAdjustable=True, basePos=basePos, mirPos=mirPos)
        self.minMount = float(minMount)
        self.maxMount = float(maxMount)
        self.scale = float(scale)
        self.offset = float(offset)
    
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        """
        length = self.lengthFromMirPos(mirPos)
        return length - self.neutralLength
    
    def mountFromPhys(self, phys):
        """Compute mount length (steps) of adjustable element given its physical length (mm)
        """
        return self.offset + (self.scale * phys)
    
    def physFromMount(self, mount):
        """Compute physical length (mm) of adjustable element given its mount length (steps)
        """
        return (mount - self.offset) / self.scale
    
#     def mountFromLength(self, length):
#         """
#         Return the mount position (steps) given the actuator length from its neutral position.
#         
#         Input: 
#         -length: length from neutral position (um)
#         
#         Output:
#         -mount: mount units (steps)
#         """
#         return self.offset + (self.scale * length)
#         
#     def lengthFromMount(self, mount):
#         """
#         Return the physical length (um) from the actuator's neutral
#         position given the mount position (steps).
#         
#         Input: 
#         -mount: mount units (steps)
# 
#         Output:
#         -length: length from neutral position (um)
#         """
#         return (mount - self.offset) / self.scale
    
    def mountInRange(self, mount):
        """Return True if the mount position is in range
        """
        return self.minMount <= mount <= self.maxMount
        
class AdjBaseActuator(AdjLengthLink):
    """Adjustable-base actuator. The base joint pistons.
    
    The direction of piston of the base is assumed to be along a vector
    pointing from basePos to mirPos at mirror neutral position.

    Right now there are three different methods to determine conversions, we should test and pick one
    (or none and just use AdjLengthActuator).
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos:  cartesian position of end of actuator fixed to the base (mm)
                    when the mirror is at zero orientation.
        - mirPos:   cartesian position of end of actuator attached to the mirror (mm)
                    when the mirror is at zero orientation.
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale:    actuator scale: mm/step
        """
        AdjLengthLink.__init__(self, basePos, mirPos, minMount, maxMount, scale, offset)
        pistonVec = self.mirPos - self.basePos
        self.pistonDir = pistonVec / numpy.linalg.norm(pistonVec)

    
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        
        This computation is exact but requires trig and so will be slow
        """
        r_not = self.neutralLength
        
        # Calculate vector from basePos to a given mirPos
        r = (mirPos - self.basePos)

        # Get projection of mirVec (link axis) along axis of motor mount.
        x = numpy.dot(r, self.pistonDir)
        
        # Get projection of mirVec (link axis) along axis perpendicular to motor mount.
        y = numpy.linalg.norm(numpy.cross(r, self.pistonDir))
        
        return x - (r_not * math.cos(math.asin(y / r_not)))

#     def physFromMirPosCCS(self, mirPos):
#         """Compute physical length (mm) of adjustable element given the mirror position (mm)
#         
#         NOTE: THIS NEEDS TO BE REWRITTEN TO NOT TAKE phys AS AN ARGUMENT
#         
#         This method makes the approximation that the projection of the 'true length'
#         along the actuator motor axis to the link axis is equal to phys.
#         This should be ok for small angles, and the solution requires no trig functions.
#         
#         Inputs:
#         - mirPos: cartesian position of end of actuator attached to the mirror (mm).
#         
#         Output:
#         - phys: approximate actuator length (mm)
#         """
#         # Calculate unit vector from basePos to a given mirPos (link axis).
#         mirVec = mirPos - self.basePos
#         mirUnitVec = mirVec / numpy.linalg.norm(mirVec)
#         
#         # Use right triangle trig to solve for truePhys value.
#         # cos(theta) = adjacent / hypotenuse
#         # theta = acos(self.pistonDir dot mirUnitVec)
#         # adjacent = phys
#         # hypotenuse = truePhys
#         truePhys = phys / numpy.dot(self.pistonDir, mirUnitVec)
#         return truePhys
               
    def physFromMirPosRO(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        
        Inputs:
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - phys: approximate actuator length (mm)
        """
        r_not = self.neutralLength
        
        # Calculate vector from basePos to a given mirPos
        r = mirPos - self.basePos

        # Get projection of mirVec (link axis) along axis of motor mount.
        x = numpy.dot(r, self.pistonDir)
        
        # Get projection of mirVec (link axis) along axis perpendicular to motor mount.
        yVec = numpy.cross(r, self.pistonDir)
        ySq = numpy.sum(yVec * yVec)
        
        return x + (ySq / (2.0 * r_not)) - r_not
                                      
class FixedLengthLink(BaseLink):
    """A fixed-length link with two ball joints: one attached to a base, the other attached to a mirror
    """
    def __init__(self, basePos, mirPos):
        BaseLink.__init__(self, isAdjustable=False, basePos=basePos, mirPos=mirPos)
    
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        """
        return 0.0
