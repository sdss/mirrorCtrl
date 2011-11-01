# renamed variables and functions to be more intuitive.
# made direct mirror able to handle encoders, if they are specified.


import numpy
import scipy.optimize  
import math
# numpy.seterr(all='raise')

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

class BaseActuator(object):
    """
    A base class for linear actuators and fixed links.
    Links are controlled in mount coordinates = steps, but all positions are in mm.
    """
    def __init__(self, isAdjustable, basePos, mirPos):
        """
        Inputs:
        - isAdjustable: Boolean. True for actuators, False for fixed links.
        - basePos:      cartesian position of end of actuator fixed to the base (mm)
        - mirPos:       cartesian position of end of actuator attached to the mirror (mm)
                          when the mirror is at neutral orientation
        """
        self.isAdjustable = bool(isAdjustable)
        self.basePos = numpy.asarray(basePos, dtype=float)
        if self.basePos.shape != (3,):
            raise RuntimeError("basePos=%s must be 3 elements" % (basePos,))
        self.mirPos = numpy.asarray(mirPos, dtype=float)
        if self.mirPos.shape != (3,):
            raise RuntimeError("mirPos=%s must be 3 elements" % (mirPos,))
        self.neutralLength = self.measureMir2Base(self.mirPos)

    def measureMir2Base(self, mirPos):
        """
        Return the distance (um), between a cartesian mirror position (mm) and the 
        cartesian base position (mm) of the actuator.
        """        
        vec = numpy.asarray(mirPos, dtype=float) - self.basePos
        length = math.sqrt(numpy.sum(vec**2) ) * (1. / MMPerMicron )
        return length
  
class LinearActuator(BaseActuator):
    """
    Linear actuator with two ball joints: one attached to a base, the
    other attached to a mirror. The actuator controls the length of the
    link between the two ball joints. This model is an estimation. True
    geometry is taken into account with the ComplexActuator object.
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos:  cartesian position of end of actuator fixed to the base (mm)
        - mirPos:   cartesian position of end of actuator attached to the mirror (mm)
                      when the mirror is at neutral orientation
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale:    actuator scale (mm/step)
        """
        BaseActuator.__init__(self, isAdjustable=True, basePos=basePos, mirPos=mirPos)
        self.minMount = float(minMount)
        self.maxMount = float(maxMount)
        self.scale = float(scale)
        self.offset = float(offset)
    
    def mountFromLength(self, length):
        """
        Return the mount position (steps) given the actuator length from its neutral position.
        
        Input: 
        -length: length from neutral position (um)
        
        Output:
        -mount: mount units (steps)
        """
       
        return self.offset + (self.scale * length)
        
    def lengthFromMount(self, mount):
        """
        Return the physical length (um) from the actuator's neutral
        position given the mount position (steps).
        
        Input: 
        -mount: mount units (steps)

        Output:
        -length: length from neutral position (um)
        """
        return (mount - self.offset) / self.scale
    
    def mountInRange(self, mount):
        """Return True if the mount position is in range
        """
        return self.minMount <= mount <= self.maxMount
        
class ComplexActuator(LinearActuator):
    """
    Actuator with two ball joints: one attached near the base, the other
    attached to a mirror. The actuator adjusts the length below the base
    ball joint while the length between ball joints remains constant.
    This differs from the LinearActuator object where the actuator
    adjusts length between the ball joints. There is a difference
    between these models because the mount motor axis is not necessarily
    aligned with the link axis between ball joints. The mount motor axis
    is defined by the vector basePos --> mirPos when the mirror is in
    the neutral position, and the link axis is defined by the vector
    basePos --> mirPos at any other mirror orientation. Methods defined
    in this object can convert the length (um) along the link axis, to a
    'true length' (um) along the actuator's motor axis. The
    ComplexActuator is a more accurate representation of an actuator
    than the LinearActuator object. Right now there are three different
    methods to determine conversions, we should test and pick one (or
    none).
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos: cartesian position of end of actuator fixed to the base (mm)
        - mirPos:  cartesian position of end of actuator attached to the mirror (mm)
                     when the mirror is at neutral orientation.
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale:    actuator scale: mm/step
        """
        LinearActuator.__init__(self, basePos, mirPos, minMount, maxMount, scale, offset)

    def doActTrigCCS(self, phys, mirPos):
        """
        This method makes the approximation that the projection of the
		'true length' along the actuator motor axis to the link axis is
		equal to phys. This should be ok for small angles, and the
		solution requires no trig functions.
        
        Inputs:
        - phys:   actuator length (um)
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - truePhys: corrected actuator length (um)
        """

        # Calculate unit vector along actuator axis of motion.
        # This coordinate system might be upside down, but it should not effect the dot product.
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # Calculate unit vector from basePos to a given mirPos (link axis).
        mirVec = mirPos - self.basePos
        mirUnitVec = mirVec / math.sqrt(numpy.dot(mirVec, mirVec))
        
        # Use right triangle trig to solve for truePhys value.
        # cos(theta) = adjacent / hypotenuse
        # theta = acos(actUnitVec dot mirUnitVec)
        # adjacent = phys
        # hypotenuse = truePhys
        truePhys = phys / numpy.dot(actUnitVec, mirUnitVec)
        return truePhys
               
    def doActTrigRO1(self, phys, mirPos):
        """ 
        This method should be the exact solution, with no
        approximations. The trig functions may slow it down.
        
        Inputs:
        - phys:   actuator length (um), not used
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - a: corrected actuator length (um)
        """
        
        r_not = self.neutralLength
        
        # Calculate vector from basePos to a given mirPos, convert to um.
        r = (mirPos - self.basePos) * (1 / MMPerMicron)

        # Calculate unit vector in direction of motor mount axis.
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # Get projection of mirVec (link axis) along axis of motor mount.
        x = numpy.dot(r, actUnitVec)
        
        # Get projection of mirVec (link axis) along axis perpendicular to motor mount.
        y = numpy.cross(r, actUnitVec)
        
        # Cross product is a vector, we want just the magnitude.
        y = math.sqrt(numpy.dot(y, y))
        
        a = x - r_not * math.cos(math.asin(y / r_not))
        
        return a
        
    def doActTrigRO2(self, phys, mirPos):
        """
		This method uses a small angle approximation to eliminate trig functions used in
        doActTrigR01.
        
        Inputs:
        - phys: actuator length (um), not used
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - a: corrected actuator length (um)
        """
        r_not = self.neutralLength
        
        # Calculate vector from basePos to a given mirPos, convert to um.
        r = (mirPos - self.basePos) * (1 / MMPerMicron)

        # Calculate unit vector in direction of motor mount axis.
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # Get projection of mirVec (link axis) along axis of motor mount.
        x = numpy.dot(r, actUnitVec)
        
        # Get projection of mirVec (link axis) along axis perpendicular to motor mount.
        y = numpy.cross(r, actUnitVec)
        
        # Cross product is a vector, we want just the magnitude.
        y = math.sqrt(numpy.dot(y, y))
        
        a = x + y**2 / (2 * r_not) - r_not
        
        return a        
                                      
class FixedLink(BaseActuator):
    """A fixed-length link with two ball joints: one attached to a base, the other attached to a mirror
    """
    def __init__(self, basePos, mirPos):
        BaseActuator.__init__(self, isAdjustable=False, basePos=basePos, mirPos=mirPos)
        
Encoder = LinearActuator

class DirectMirror(object):
    """
    A Direct Mirror Object. All actuators connected directly to mirror. If an encoderList is not
    specified, the default action is to use the actuatorList as the encoderList.
    """
    def __init__(self, actuatorList, encoderList=None, rotZConst = bool(False)):
        """
        Inputs:
        - actuatorList: 6 item list of actuator or fixed link objects.
							This list is used when determining mount units from a given
							orientation (see orient2Mount method).
        - encoderList:  (optional) 6 item list of encoder objects, 1 for each actuator. If an 
        			        actuator has no respective encoder, enter 'None'. In the case 
        			        of 'None', the actuator is substituted for an encoder. This list is 
        			        used when determining and orientation from a given set of mount
        			       coordinates (see mount2Orient method).
        rotZConst: 		Boolean. Set to True if Z rotation is constrained by a fixed link.
        				    this could be later generalized to allow constraints on any axis.
        """
        if len(actuatorList) != 6:
            raise RuntimeError("Need exactly 6 actuators; %s supplied" % (len(actuatorList,)))
        self.actuatorList = actuatorList   # for orient2mount
        if encoderList == None:
            self.encoderList = actuatorList  # for mount2orient
        else:
            if len(encoderList) != 6:
                raise RuntimeError("encoderList=%s must be 6 elements, insert 'None' in list if \n\
                                    there is a 'missing' encoder" % len(encoderList))
        
            # Populate encoderList with encoders, using actuators in place of 'None' encoder.
            self.encoderList = []
            for ind, enc in enumerate(encoderList):
                if enc == None:
                    self.encoderList.append(actuatorList[ind])
                else:
                    self.encoderList.append(enc)
        self.rotZConst = rotZConst

              
    def _mount2OrientMin_func(self, orient, g_deltaPhys, g_deltaPhysMult, actuatorList):
        """
        This method is iterated over in mount2Orient using Powell's method.     
        Inputs: 
        -orient[0:5]:            6 axis orientation.
        -g_deltaPhys[0:n-1]:     Computed from given mount coords. n = num of actuators.
        -g_deltaPhysMult[0:n-1]: Computed errors. n = num of actuators.
        
        Output: 1 + sum(deltaPhysMult * deltaPhysErr ** 2)
        """       
        # called only from mount2Orient
        deltaPhys = self._orient2DeltaPhys(orient, actuatorList)
        deltaPhysErr = (deltaPhys - g_deltaPhys) ** 2
        return 1 + sum(g_deltaPhysMult * deltaPhysErr)
        
    def _orient2RotTransMats(self, orient):
        """
        This function computes the rotation matrices and translation
        offsets from a given orientation. The outputs are used to
        transform the cartesian coordinates of actuators based on the
        orientation. 
        
        Input:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation (")     
        - actuatorList: 6 element list of LinearActuator or Encoder objects.

        Outputs: 
        - rotMatXY: 3x3 rotation matrix defined from x-y tilts
        - rotMatZ:  2x2 rotation matrix defined by the z rotation angle. A rotation in the xy plane.
        - offsets:  3x1 (x,y,z) offset vector
        """
        # Compute values in mm and degrees.
        pist    = orient[0] * MMPerMicron
        tiltX   = orient[1] * RadPerArcSec
        tiltY   = orient[2] * RadPerArcSec
        transX  = orient[3] * MMPerMicron
        transY  = orient[4] * MMPerMicron
        rotZ    = orient[5] * RadPerArcSec
        
        if self.rotZConst == True:
            # Z is held constant, so rotZ should, and must equal 0 and thus ignored.
            # Set rotZ to 0, just incase it wasn't (eg if a rotation was commanded for a
            # non-rotating mirror.
            rotZ = 0.
                
        cosX = math.cos(tiltX)
        sinX = math.sin(tiltX)
        cosY = math.cos(tiltY)
        sinY = math.sin(tiltY)
        rotMatX = numpy.array([ [1,    0,     0],
                                [0, cosX, -sinX],
                                [0, sinX,  cosX] ])
                                
        rotMatY = numpy.array([ [cosY,  0, sinY],
                                [0,     1,    0],
                                [-sinY, 0, cosY] ])

        # Create single vector rotation matrix, rotate by X first, Y second
        rotMatXY = numpy.dot(rotMatY, rotMatX) # this is right

        sinZ = math.sin(rotZ)
        cosZ = math.cos(rotZ)
        rotMatZ = numpy.array([ [cosZ, -sinZ],
                                [sinZ,  cosZ] ])

        offsets = numpy.array([transX, transY, pist])
        
        return rotMatXY, rotMatZ, offsets
        
    def _orient2DeltaPhys(self, orient, actuatorList):
        """
        Translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation (")     
        - actuatorList: 6 element list of LinearActuator or Encoder objects.

        Output: 
        - deltaPhysList[0:5]: delta length for actuators (um) measured from the neutral position.
        				        FixedLink actuators always return a deltaPhys of 0 because they
        				        cannot change length.
        """

        # Get rotation matrices and offsets.
        rotMatXY, rotMatZ, offsets = self._orient2RotTransMats(orient)
        
		# list of acutator outputs in physical length (um), with neutral as the zero point.
        deltaPhysList = []   
        for act in actuatorList:
            # Just doing standard case  begining on line 228 of oneorient2phys.for
            
            # First rotate mirror end of actuator about the mirror vertex
            # (do this before translation so the vertex is at 0,0,0)
            actUnrot = act.mirPos
            
            # Rotate 3-vector
            actMirPos = numpy.dot(rotMatXY, actUnrot)
           
            
            # Rotate xy coordinate system about z axis
            actMirPos[0:2] = numpy.dot(rotMatZ, actMirPos[0:2]) 
            
            # Apply all translations to compute final positions of
            # the mirror end gimbals of the actuators.
            actMirPos = actMirPos + offsets
            
            if act.isAdjustable:
                deltaPhys = act.measureMir2Base(actMirPos) - act.neutralLength  
                               
                try:
                    # compute new phys if act is a ComplexActuator. Choose your favorite 
                    # method.
                    deltaPhys = act.doActTrigRO2(deltaPhys, actMirPos)
                except AttributeError:
                    pass
            else:
                 deltaPhys = 0. # no change in actuator length
            deltaPhysList.append(deltaPhys)        
        return numpy.asarray(deltaPhysList, dtype=float)

    def _mount2Orient(self, mount):
        """
        Translated from: src/subr/mir/onemount2orient.for. This is a
        'private' method, to allow for small adjustments to be made in
        other mirror types.
        
        Input:
        - mount[0:n]: mount position for n actuators or encoders. 
       
        Output:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation (")
        
        """
        mount = numpy.asarray(mount, dtype=float)
        
        # We read from encoders to compute orientation.
        actuatorList = self.encoderList
        # First remove fixed links if present, they will not be used in initial minimization
        for ind, act in enumerate(actuatorList):
            if act.isAdjustable == False:
                del actuatorList[ind]
                
        actNum = len(actuatorList)        
        # Compute physical errors
        maxOrientErr = numpy.array([1., 0.001, 0.001, 0.1, 0.1, 1.])
        
        # compute physical position at zero orientation, won't this always be zero?!?
        # why compute it?
        orient  = numpy.zeros((6,))
        deltaPhysAtZero = self._orient2DeltaPhys(orient, actuatorList)
        
        # Compute delta physical length at perturbed orientations
        # Sum the square of errors
        # Orient is zeros except one axis on each iteration.
        
        # diag matrix for extracting rows with a single non-zero element
        orient = numpy.diag(maxOrientErr,0) 
        
        maxDeltaPhysErr = numpy.zeros(actNum)
        for pert in orient:
            deltaPhys = self._orient2DeltaPhys(pert, actuatorList)
            # sum over perturbations
            maxDeltaPhysErr = maxDeltaPhysErr + (deltaPhys - deltaPhysAtZero) ** 2   
        

        g_deltaPhysMult = 1. / maxDeltaPhysErr
        g_deltaPhys = []
        for ind, act in enumerate(actuatorList):
            g_deltaPhys.append(act.lengthFromMount(mount[ind]))
        # initial guess ("home")
        initOrient = numpy.zeros(6)
        fitTol = 1e-8
        maxIter = 10000
        orient = scipy.optimize.fmin_powell(self._mount2OrientMin_func, initOrient, 
                                    args=(g_deltaPhys, g_deltaPhysMult, actuatorList), 
                                    maxiter=maxIter, ftol = fitTol) 
                                    
        return orient
        
    def mount2Orient(self, mount):
        """
        In the case of a direct mirror with 6 actuators, this is the same function as _mount2Orient
        """
        orient = self._mount2Orient(mount)
        return orient
        
    def orient2Mount(self, orient):
        """
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation (")

        Output: 
        - mount[0:n-1] = mount position for n movable actuators
        """
        
        # We read from actuators to compute mount positions
        actuatorList = self.actuatorList
        deltaPhysList = self._orient2DeltaPhys(orient, actuatorList)
        mountList = []
        for ind, act in enumerate(actuatorList):
            if act.isAdjustable:
                mount = act.mountFromLength(deltaPhysList[ind])
                if act.mountInRange(mount) != 1:
                    raise RuntimeError('mount=%f, out of range for actuator %i' % (mount,counter))
                mountList.append(mount)
            # should we return a mount value for a FixedLink?  I am not. If you want it uncomment:
            #else:
                 #mountList.append(0.) # fixed link, no mount change
        return numpy.asarray(mountList, dtype=float)
        
class DirectMirrorZFix(DirectMirror):
    """
    A class of mirror with a FixedLink constraining Z rotation. The only
    added feature of this object is that an extra optimization is used
    to incorporate the residual z rotation induced from the fixed link
    at a given mirror orientation.
    """

    def __init__(self, actuatorList, encoderList = None, rotZConst = bool(True)):
        DirectMirror.__init__(self, actuatorList = actuatorList, encoderList = encoderList,
                               rotZConst = rotZConst)
        
    def _zRotMin_func(self, Z, orient, act):
        """
        This method is iterated over to determine the best Z rotation solution
        when there is a fixedLink constraining Z rotation. We minimize:
        (measuredLinkLength - knownLinkLength)^2, there should only be one correct answer.
        In reality this answer needs to be zero.
   
        Inputs:
        - Z:      z rotation angle in ArcSec. Parameter to be optimized.
        - orient: 6 axis orientation with Z rotation set to zero, this is 
        			the output of mount2Orient.         
        - act: 	  The FixedLink, with a length that is conserved.
        
        Outputs: 1 + (measuredLinkLength - knownLinkLength)^2, thus the minimum
        		   value we seek is 1.
        """

        orient[5] = Z
        rotMatXY, rotMatZ, offsets = self._orient2RotTransMats(orient)
        actUnrot = act.mirPos
        
        # rotate 3-vector
        actMirPos = numpy.dot(rotMatXY, actUnrot)
        
        # rotate xy coordinate system about z axis
        actMirPos[0:2] = numpy.dot(rotMatZ, actMirPos[0:2])
        # apply all translations to compute final positions of
        # the mirror end gimbals of the actuators
        actMirPos = actMirPos + offsets
        deltaPhys = act.measureMir2Base(actMirPos) - act.neutralLength  
        # in reality, deltaPhys = 0, because the measured distance between the fixed link's
        # mirPos and basePos should always equal the neutralLength. We are using this fact to
        # minimize the difference between measured length and expected length by varying Z
        # to find the best solution.
        return 1 + deltaPhys**2        

    def _computeZRot(self, orient):
        """
        For a given oriention, find the correct Z rotation value taking into account the FixedLink
        constraining the Z rotation.
        """
        # Take Z rotation fixed link into account when computing orientation.
        # Although this mirror is constrained in Z rotation, we will turn the constraint off
        # to compute the small amount of rotation that is induced when the mirror orientation
        # changes.
        self.rotZConst = False

        # Get fixed link constraining Z rotation, this is the only one that isn't adjustable
        actuatorList = self.encoderList
        for act in actuatorList:
            if act.isAdjustable == False:               
				fitTol = 1e-8
				maxIter = 10000
				zRot = scipy.optimize.fmin_powell(self._zRotMin_func, 0., 
													args=(orient, act),
													maxiter=maxIter, ftol=fitTol)
				# Put the freshly computed zRot back into orient
				orient[5] = zRot
        
        # Turn Z constraint back on.
        self.rotZConst = True
        return orient
    
    def mount2Orient(self, mount):
    	"""
    	Most of the work is done in _mount2Orient, which will return with a z rotation of 0.
    	The actual z rotation induced by the FixedLink is then computed using _computeZRot.
    	
        Input:
        - mount[0:n]: mount position for n actuators or encoders. 
       
        Output:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation ("). Non-zero!
 
    	"""
        orient = self._mount2Orient(mount)
        orient = self._computeZRot(orient)
        return orient
        
class TipTransMirror(DirectMirror):
    """
    Tip-Trans Mirror. Translate by tipping a central linear bearing.
    orient2phys method is different, other methods are same as
    DirectMirror obj.
    """
    def __init__(self, ctrMirZ, ctrBaseZ, actuatorList, encoderList = None):
        DirectMirror.__init__(self, actuatorList, encoderList = encoderList)
        self.ctrMirZ = ctrMirZ
        self.ctrBaseZ = ctrBaseZ
        
    def _rotEqPolMat(self, eqAng, polAng):
        """
        This method was translated from src/subr/cnv/roteqpol.for.
        Defines a matrix that rotates a 3-vector described by equatorial
        and polar angles as follows: The plane of rotation contains the
        z axis and a line in the x-y plane at angle eqAng from the x
        axis towards y. The amount of rotation is angle polAng from the
        z axis towards the line in the x-y plane.
        
        Inputs:
        - eqAng:  Equatorial rotation angle (radians) of line in x-y plane (from x to y).
                   This plane of rotation includes this line and z.
        - polAng: Polar rotation angle (radians) from z axis to the line in the x-y plane.
        
        Output:
        - rotMatEqPol: 3x3 rotation matrix
        """ 
        sinEq = math.sin(eqAng) 
        cosEq = math.cos(eqAng)
        sinPol = math.sin(polAng)
        cosPol = math.cos(polAng)

        rotMatEqPol = numpy.array([ [ sinEq**2 + (cosEq**2 * cosPol), 
                                  -sinEq * cosEq * (1 - cosPol),
                                                 cosEq * sinPol ],
                                                
                               [  -sinEq * cosEq * (1 - cosPol),
                                 cosEq**2 + (sinEq**2 * cosPol),
                                                 sinEq * sinPol ],
                                                
                               [                -cosEq * sinPol,
                                                -sinEq * sinPol,
                                                         cosPol ]  ])  
        return rotMatEqPol
                                
    def _orient2DeltaPhys(self, orient, actuatorList):
        """
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        - orient[0:5]:  mirror orientation with 6 axes 6 item list:
        - orient[0]:    piston (um)
        - orient[1:3]:  x-y tilt (")
        - orient[3:5]:  x-y translation (um)
        - orient[5]:    z rotation (")     
        - actuatorList: 6 element list of LinearActuator or Encoder objects.

        Output: 
        - deltaPhysList[0:5]: delta length for actuators (um) measured from the neutral position.
        				        FixedLink actuators always return a deltaPhys of 0 because they
        				        cannot change length.
        """

        # starting from line 152 in oneOrient2Phys.for (special case = tiptransmir)
        
        # Determine ctrMirPos, the final position of the central linear
        # bearing mirror gimbal. This gimbal is fixed to the
        # mirror (at ctrMirZ from the vertex) and slides along
        # the central linear bearing.
        # First rotate the gimbal position about the vertex
        # (do this before translation so the vertex is at 0,0,0)
        
        ctrMirZ = self.ctrMirZ
        ctrBaseZ = self.ctrBaseZ
        ctrUnrot = numpy.zeros(3)
        ctrUnrot[2] = ctrMirZ
        

        rotMatXY, rotMatZ, offsets = self._orient2RotTransMats(orient)
        ctrMirPos = numpy.dot(rotMatXY, ctrUnrot)
        
        # Now apply translation to produce the final position.
        ctrMirPos = ctrMirPos + offsets
        # Determine ctrBasePos, the position of the central linear
        # bearing base gimbal. This gimbal attaches the central linear
        # bearing to the frame. Its position is fixed; it does not
        # change as the mirror moves around.
        ctrBasePos = numpy.zeros(3)
        ctrBasePos[2] = ctrBaseZ
        # Determine the vector from base gimbal to mirror gimbal.
        ctrExtent = ctrMirPos - ctrBasePos
        # Determine the new positions of the transverse actuators
        # at the transverse gimbal end. This gimbal sets the tilt
        # of the central linear bearing and is at a fixed length
        # from the base gimbal. The home position of this end of the
        # transverse actuators is actMirPos (despite the name).
        #
        # To do this, rotate the home position about the central linear
        # bearing base pivot (ctrBasePos) by the amount the bearing tips.
        # Note: rotation occurs in a plane defined by the motion
        # of the central linear bearing, and is by the amount
        # the bearing tips, so this is an "equatorial/polar" rotation
        # (solved by nv_RotEqPol). I think. Close enough, anyway.
        # First compute the equatorial and polar angle.
        ctrLenXY = math.sqrt(numpy.sum(ctrExtent[0:2] ** 2))
        if ctrExtent[2] > 0.:
            eqAng = math.atan2(ctrExtent[1], ctrExtent[0])
        else:
            eqAng = math.atan2(-ctrExtent[1], -ctrExtent[0])
        polAng = math.atan2(ctrLenXY, numpy.abs(ctrExtent[2]))
        # compute home position of transverse actuators
        # at transverse gimbal end, with respect to the base gimbal
        
        # make eq-pol rotation matrix
        rotMatEq = self._rotEqPolMat(eqAng, polAng)
       
        deltaPhysList = []
        for act in actuatorList:
            actUnrot = numpy.asarray(act.mirPos, dtype=float) - ctrBasePos
            # rotate this about the base pivot to compute the actual position
            # of the transverse actuators at the transverse gimbal end,
            # with respect to the base pivot
            actMirPos = numpy.dot(rotMatEq, actUnrot)
            # compute the position of the transverse actuators at the
            # transverse gimbal end to the standard reference frame
            # (which is with respect to home position of mirror vertex).
            actMirPos = actMirPos + ctrBasePos
            if act.isAdjustable:
                try:
                    # compute new phys if act is a ComplexActuator
                    deltaPhys = act.doActTrigRO2(deltaPhys, actMirPos)
                except AttributeError:
                    pass                        
                deltaPhys = act.measureMir2Base(actMirPos) - act.neutralLength    
                # phys units: um
            else:
                deltaPhys = 0.  # no change in actuator length (FixedLink)
            deltaPhysList.append(deltaPhys)
        return numpy.asarray(deltaPhysList, dtype=float)