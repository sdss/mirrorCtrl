"""Links

Mirrors are constrained by a set of 6 links: a mix of actuators and fixed-length links.

Links have two pivot points that are assumed to be perfect ball joints:
one attached to the mirror, the other attached to the mirror support structure.
These are defined by cartesian coordinates mirPos and basePos.

Links are either adjustable (actuators or encoders) or have fixed length.
Important attributes of adjustable links:
- physical length: length of actuator/sensor in mm
    0 when the mirror is at zero orientation
    + makes the actuator longer
- mount length: length of actuator/sensor in native units: microsteps or ticks;
    this differs from mount by an offset and a scale:
    mount = offset + (scale * phys)
- length: the distance from the base ball joint to the mirror ball joint

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
        @param[in isAdjustable: True for actuators, False for fixed links.
        @param[in] basePos: cartesian position of end of actuator fixed to the base (mm)
        @param[in] mirPos:  cartesian position of end of actuator attached to the mirror (mm)
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
        """Compute desired physical length (mm) of adjustable element given the mirror position (mm)

        @param[in] mirPos: 3 element list, describing the [x,y,z] position.
        The link may not be capable of achieving this length.
        
        Subclasses must override.
        """
        raise NotImplementedError("Subclasses must define")

    def lengthFromMirPos(self, mirPos):
        """
        Return the desired length of link in mm.
        
        @param[in] mirPos: 3 element numpy.array, describing the [x,y,z] mirror position.

        This is the distance between the supplied mirPos and the original basePos.
        The link may not be capable of achieving this length.
        """        
        return numpy.linalg.norm(self.basePos - mirPos)


class FixedLengthLink(BaseLink):
    def __init__(self, basePos, mirPos):
        """A fixed-length link with two ball joints: one attached to a base, the other attached to a mirror

        @param[in] mirPos: 3 element list-like, describing the [x,y,z] position of link attachment to mirror (in neutral position)
        @param[in] basePos: 3 element list-like, describing the [x,y,z] position of link base
        """
        BaseLink.__init__(self, isAdjustable=False, basePos=basePos, mirPos=mirPos)
    
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        """
        length = self.lengthFromMirPos(mirPos)
        return length - self.neutralLength


class AdjustableLink(BaseLink):
    """Generic adjustable-length actuator: a base class for specific designs.
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """

        @param[in] basePos:  cartesian position of end of actuator fixed to the base (mm)
        @param[in] mirPos:   cartesian position of end of actuator attached to the mirror (mm)
                    when the mirror is at zero orientation
        @param[in] minMount: minimum length (steps)
        @param[in] maxMount: maximum length (steps)
        @param[in] scale:    actuator scale (mount units/um)
        @param[in] offset:   mount position when mirror is at zero orientation
        """
        BaseLink.__init__(self, isAdjustable=True, basePos=basePos, mirPos=mirPos)
        self.minMount = float(minMount)
        self.maxMount = float(maxMount)
        self.scale = float(scale)
        self._scale = float(scale) * 1000. # convert scale to mount units/mm
        self.offset = float(offset)

    def mountFromPhys(self, phys):
        """Compute mount length (steps) of adjustable element given its physical length (mm)

        @param[in] phys: length in mm
        @return mount: length in steps
        """
        return self.offset + (self._scale * phys)
    
    def physFromMount(self, mount):
        """Compute physical length (mm) of adjustable element given its mount length (steps)

        @param[in] mount: length in steps
        @return pyhs: length in mm
        """
        return (mount - self.offset) / (self._scale)
    
    def mountInRange(self, mount):
        """Return True if the mount position is in it's allowed range
        @param[in] mount: mount units
        @return bool
        """
        return self.minMount <= mount <= self.maxMount


class AdjLengthLink(AdjustableLink):
    """Adjustable-length actuator in which the distance between the two ball joints varies.
    """
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        @param[in] mirPos: xyz position
        @return phys: physical length of the link in mm
        """
        length = self.lengthFromMirPos(mirPos)
        return length - self.neutralLength


class AdjBaseActuator(AdjLengthLink):
    def __init__(self, *args, **kwargs):
        """Adjustable-base actuator. The base joint pistons.
        
        The direction of piston of the base is assumed to be along a vector
        pointing from basePos to mirPos at mirror neutral position.

        Right now there are three different methods to determine conversions, we should test and pick one
        (or none and just use AdjLengthActuator).
        """
        AdjLengthLink.__init__(self, *args, **kwargs)
        # Unit vector along the axis of motion of the actuator base
        self.pistonDir = (self.mirPos - self.basePos) / numpy.linalg.norm((self.mirPos - self.basePos))
        
#     def physFromMirPos(self, mirPos):
#         """Compute physical length (mm) of adjustable element given the mirror position (mm)
#         
#         This computation is exact but requires trig and so will be slow
#         """
#         # this method is producing math domain errors, I think it has to do 
#         # with machine precision
#         r_naught = self.neutralLength
#         
#         # Calculate vector from basePos to a given mirPos
#         r = (mirPos - self.basePos)
#         # Get projection of mirVec (link axis) along axis of motor mount.
#         x = numpy.dot(r, self.pistonDir)
#         
#         y = numpy.linalg.norm(numpy.cross(r, self.pistonDir))
#         try: 
#             return x - (r_naught * math.cos(math.asin(y / r_naught)))
#         except ValueError:
#             raise RuntimeError( 'Err: lim of y = r_naught, but y=%5.3f r_n=%5.3f' % (y, r_naught))
               
    def physFromMirPos(self, mirPos):
        """Compute physical length (mm) of adjustable element given the mirror position (mm)
        
        @param[in] mirPos: cartesian position of end of actuator attached to the mirror (mm).
        @param[in] phys: approximate actuator length (mm)
        @return phys: physical length in mm
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
