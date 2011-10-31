# ccs: - broke out _orient2phys() method because it is necessary for both mount<-->orient
#            conversions
#           - fixed orient2mount/phys to return mountList, not mount, and as numpy array
#           - changed the shape checks from [3] to (3,)
#           - mountOffset added in LinearActuator class.
#           - changing russells length units to um
#           - adding DirectMir + Encoder class
#           - broke lin alg into functions, but now they are recomputed each loop, slow 
#               but readable.  could just compute rotation matrices outside loop and dot    
#               inside loop
#           - ctrMirZ needs to be defined somewhere for tip trans mirs, i'll put it in a slot
#           - moved complex actuator trig out of parent class functions
#           - recomment at 361
#           - optimizing
#           - how do we want to handle fixed links commanded to move?
"""
Problems: 
encoderList can overwrite actuatorList
mount depends on computed mirPos for actuators but not for encoders?
"""


import numpy
import scipy.optimize  
import math
# numpy.seterr(all='raise')

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

class BaseActuator(object):
    """A base class for linear actuators and fixed links.
    Links are controlled in mount coordinates = steps, but all positions are in mm
    """
    def __init__(self, isAdjustable, basePos, mirPos):
        """
        Inputs:
        - basePos: cartesian position of end of actuator fixed to the base (mm)
        - mirPos: cartesian position of end of actuator attached to the mirror (mm)
            when the mirror is at neutral orientation
        """
        self.isAdjustable = bool(isAdjustable)
        self.basePos = numpy.asarray(basePos, dtype=float)
        if self.basePos.shape != (3,):
            raise RuntimeError("basePos=%s must be 3 elements" % (basePos,))
        self.mirPos = numpy.asarray(mirPos, dtype=float)
        if self.mirPos.shape != (3,):
            raise RuntimeError("mirPos=%s must be 3 elements" % (mirPos,))
        self.neutralLength = self.getLength(self.mirPos)

    def getLength(self, mirPos):
        """Return the length of the actuator (um)
        """        
        vec = numpy.asarray(mirPos, dtype=float) - self.basePos
        length = math.sqrt(numpy.sum(vec**2) ) * (1. / MMPerMicron )
        return length
  
class LinearActuator(BaseActuator):
    """Linear actuator with two ball joints: one attached to a base, the other attached to a mirror    
    The actuator controls the length of the link between the two ball joints. This model is an
    estimation. True geometry is taken into account with the ComplexActuator object.
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos: cartesian position of end of actuator fixed to the base (mm)
        - mirPos: cartesian position of end of actuator attached to the mirror (mm)
            when the mirror is at neutral orientation
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale: actuator scale: mm/step
        """
        BaseActuator.__init__(self, isAdjustable=True, basePos=basePos, mirPos=mirPos)
        self.minMount = float(minMount)
        self.maxMount = float(maxMount)
        self.scale = float(scale)
        self.offset = float(offset)
    
    def mountFromLength(self, length):
        """Return the mount position (steps) given the length tuple.
        
        input: length is a tuple containing (length (um), unit vec (x,y,z))
        """
       
        return self.offset + (self.scale * length)
        
    def lengthFromMount(self, mount):
        """Return the physical length (um) given the mount position (steps)
        """
        return (mount - self.offset) / self.scale
    
    def mountInRange(self, mount):
        """Return True if the mount position is in range
        """
        return self.minMount <= mount <= self.maxMount
        
class ComplexActuator(LinearActuator):
    """Actuator with two ball joints: one attached near the base, the other attached to a mirror.    
    The actuator controls the length of the mirror ball joint and the base. This is a more accurate
    representation of an actuator than the LinearActuator object. The mount motor axis is not
    necessarily aligned with the link between ball joints. This object also accounts 
    for an actuator with a motor mount axis that is NOT normal to the XY plane. 
    """
    def __init__(self, basePos, mirPos, minMount, maxMount, scale, offset):
        """
        Inputs:
        - basePos: cartesian position of end of actuator fixed to the base (mm)
        - mirPos: cartesian position of end of actuator attached to the mirror (mm)
            when the mirror is at neutral orientation
        - minMount: minimum length (steps)
        - maxMount: maximum length (steps)
        - scale: actuator scale: mm/step
        """
        LinearActuator.__init__(self, basePos, mirPos, minMount, maxMount, scale, offset)

    def doActTrigCCS(self, phys, mirPos):
        """
        This function takes the delta length (um) of the actuator, and returns the 
        'true length', which is dependent on the geometry of the actuator. This conversion
        accounts for the fact that the length between the ball joints is not changing, but the
        length between the actuator base and the lower ball joint. Assumes actuator axis of motion
        is aligned with the vector between basePos and mirPos in the mirror's neutral position.
        This method makes the approximation that the change in distance from basePos to mirPos
        is equal to the projection of the mount change onto the basePos-mirPos axis. This 
        should be ok for small angles, and the solution requires no trig functions.
        
        Inputs:
        - phys: actuator length (um)
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - truePhys: corrected actuator length (um)
        """

        # calculate unit vector along actuator axis of motion.
        # this coordinate system might be upside down.  To fix reverse order of vector subtraction.
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # calculate unit vector from basePos to a given mirPos
        mirVec = mirPos - self.basePos
        mirUnitVec = mirVec / math.sqrt(numpy.dot(mirVec, mirVec))
        
        # use right triangle trig to solve for truePhys value
        # cos(theta) = adjacent / hypotenuse
        # theta = acos(actUnitVec dot mirUnitVec)
        # adjacent = phys
        # hypotenuse = truePhys
        truePhys = phys / numpy.dot(actUnitVec, mirUnitVec)
        return truePhys
               
    def doActTrigRO1(self, phys, mirPos):
        """
        This function returns the 
        'true length', which is dependent on the geometry of the actuator. This conversion
        accounts for the fact that the length between the ball joints is not changing, but the
        length between the actuator base and the lower ball joint. Assumes actuator axis of motion
        is aligned with the vector between basePos and mirPos in the mirror's neutral position. 
        This method should be the exact solution, with no approximations.
        
        Inputs:
        - phys: actuator length (um), not used
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - a: corrected actuator length (um)
        """
        
        r_not = self.neutralLength
        
        # calculate vector from basePos to a given mirPos, convert to um
        r = (mirPos - self.basePos) * (1 / MMPerMicron)

        # calculate unit vector in direction of motor mount
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # get projection of mirVec along axis of motor mount
        x = numpy.dot(r, actUnitVec)
        
        # get projection of mirVec along axis perpendicular to motor mount
        y = numpy.cross(r, actUnitVec)
        
        # cross product is a vector, we want just the magnitude
        y = math.sqrt(numpy.dot(y, y))
        
        a = x - r_not * math.cos(math.asin(y / r_not))
        
        return a
        
    def doActTrigRO2(self, phys, mirPos):
        """
        This function takes the delta length (um) of the actuator, and returns the 
        'true length', which is dependent on the geometry of the actuator. This conversion
        accounts for the fact that the length between the ball joints is not changing, but the
        length between the actuator base and the lower ball joint. Assumes actuator axis of motion
        is aligned with the vector between basePos and mirPos in the mirror's neutral position.
        This method uses a small angle approximation to eliminate trig functions used in
        doActTrigR01.
        
        Inputs:
        - phys: actuator length (um), not used
        - mirPos: cartesian position of end of actuator attached to the mirror (mm).
        
        Output:
        - a: corrected actuator length (um)
        """
        r_not = self.neutralLength
        
        # calculate vector from basePos to a given mirPos, convert to um
        r = (mirPos - self.basePos) * (1 / MMPerMicron)

        # calculate unit vector in direction of motor mount
        actVec = self.mirPos - self.basePos
        actUnitVec = actVec / math.sqrt(numpy.dot(actVec, actVec))
        
        # get projection of mirVec along axis of motor mount
        x = numpy.dot(r, actUnitVec)
        
        # get projection of mirVec along axis perpendicular to motor mount
        y = numpy.cross(r, actUnitVec)
        
        # cross product is a vector, we want just the magnitude
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
    A Direct Mirror Object. All actuators connected directly to mirror
    """
    
    def __init__(self, actuatorList):
        if len(actuatorList) != 6:
            raise RuntimeError("Need exactly 6 actuators; %s supplied" % (len(actuatorList,)))
        self.forwardList = actuatorList   # for orient2mount
        self.backwardList = actuatorList  # for mount2orient
        
    def _zRotMount2Orient_func(self, Z, orient, act):
        """
        This method is iterated over to determine the best Z rotation solution
        when there is a fixedLink constraining Z rotation. We minimize:
        (measuredLinkLength - knownLinkLength)^2, there should only be one correct answer.
        Much was taken from _orient2Phys.
        
        inputs:
        Z: parameter to be optimized, z rotation angle in ArcSecPerDeg
        orient: Z rot is set to zero, this is the output of mount2Orient.
            [pist (um), tiltX(arcsec), tiltY(arcsec), transX(arcsec), transY(arcsec), 0.]
        act: the FixedLink, with a length that is conserved
        
        outputs: (measuredLinkLength - knownLinkLength)^2
        """
        
        pist    = orient[0] * MMPerMicron
        tiltX   = orient[1] * RadPerArcSec
        tiltY   = orient[2] * RadPerArcSec
        transX  = orient[3] * MMPerMicron
        transY  = orient[4] * MMPerMicron
        rotZ    = Z * RadPerArcSec
        
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

        # create single vector rotation matrix, rotate by X first, Y second
        rotMatXY = numpy.dot(rotMatY, rotMatX) # this is right

        sinTheta = math.sin(rotZ)
        cosTheta = math.cos(rotZ)
        rotMatZ = numpy.array([ [cosTheta, -sinTheta],
                                [sinTheta,  cosTheta] ])

        
        # First rotate mirror end of actuator about the mirror vertex
        # (do this before translation so the vertex is at 0,0,0)
        actUnrot = act.mirPos
        
        # rotate 3-vector
        actMirPos = numpy.dot(rotMatXY, actUnrot)
       
        
        # rotate xy coordinate system about z axis
        actMirPos[0:2] = numpy.dot(rotMatZ, actMirPos[0:2]) # recomputes eack loop, slow but readable
        
        # apply all translations to compute final positions of
        # the mirror end gimbals of the actuators
        actMirPos = actMirPos + numpy.array([transX, transY, pist])
        phys = act.getLength(actMirPos) - act.neutralLength  
        
        return phys**2        
        
    def _mount2Orient_func(self, orient, g_phys, g_physMult):
        """
        This method is iterated over in mount2Orient using powell's method        
        inputs: 
        -orient[0:5]
        -g_phys: computed from given mount coords
        -g_physMult: computed errors
        
        output: 1 + sum(physMult * physErr ** 2)
        """       
        # called only from mount2Orient
        actuatorList = self.backwardList  
        phys = self._orient2Phys(orient, actuatorList)
        fixLinkInd = ~numpy.isnan(phys)
        phys = phys[fixLinkInd]  # remove fixed links from calculation, bad
        #phys = numpy.nan_to_num(phys) # fixed links
        physErr = (phys - g_phys) ** 2
        return 1 + sum(g_physMult * physErr)
        
    def _rotXY(self, inVec, tiltX, tiltY):
        """
        This method was translated from src/subr/cnv/rotxy.for
        inputs:
        - inVec: a 3 vector to be transformed
        - tiltX: x axis tilt in radians
        - tiltY: y axis tilt in radians
        
        output:
        - outVec: a 3 vector that has been transformed by a rotation in X and Y.
        """
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

        # create single vector rotation matrix, rotate by X first, Y second
        rotMatXY = numpy.dot(rotMatY, rotMatX) # this is right
        outVec = numpy.dot(rotMatXY, inVec)
        return outVec
        
    def _rot2D(self, inXY, theta):
        """
        This function was translated from src/subr/cnv/rot2d
        inputs:
        - inXY[0:1]: tuple containing input 2D coord (x, y)
        - theta: rotation angle (radians)
        
        outputs:
        - outXY: tuple containing rotated 2D coord (x, y)
        """

        sinTheta = math.sin(theta)
        cosTheta = math.cos(theta)
        rotMat = numpy.array([ [cosTheta, -sinTheta],
                               [sinTheta,  cosTheta] ])
                                
        outXY = numpy.dot(rotMat, inXY)
        return outXY
    
    def _orient2Phys(self, orient, actuatorList):
        """
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        orient[0:5] = mirror orientation with 6 axes 6 item list:
        orient[0] = piston (um)
        orient[1:3] = x-y tilt (")
        orient[3:5] = x-y translation (um)
        orient[5] = z rotation (")     
        actuatorList = 6 element list of LinearActuator or Encoder objects.

        Output: 
        physList[0:n-1] =  delta length for n actuators, is n=6 necessary?
        """
        # add exception for size of orient?
        
        # compute values in mm and degrees
        pist    = orient[0] * MMPerMicron
        tiltX   = orient[1] * RadPerArcSec
        tiltY   = orient[2] * RadPerArcSec
        transX  = orient[3] * MMPerMicron
        transY  = orient[4] * MMPerMicron
        rotZ    = orient[5] * RadPerArcSec
        
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

        # create single vector rotation matrix, rotate by X first, Y second
        rotMatXY = numpy.dot(rotMatY, rotMatX) # this is right

        sinTheta = math.sin(rotZ)
        cosTheta = math.cos(rotZ)
        rotMatZ = numpy.array([ [cosTheta, -sinTheta],
                               [sinTheta,  cosTheta] ])

        physList = []   # list of acutator outputs in physical length (um); None for fixed links
        for act in actuatorList:
            # Just doing standard case  begining on line 228 of 
            # oneorient2phys.for
            
            # First rotate mirror end of actuator about the mirror vertex
            # (do this before translation so the vertex is at 0,0,0)
            actUnrot = act.mirPos
            
            # rotate 3-vector
            actMirPos = numpy.dot(rotMatXY, actUnrot)
           
            
            # rotate xy coordinate system about z axis
            actMirPos[0:2] = numpy.dot(rotMatZ, actMirPos[0:2]) # recomputes eack loop, slow but readable
            
            # apply all translations to compute final positions of
            # the mirror end gimbals of the actuators
            actMirPos = actMirPos + numpy.array([transX, transY, pist])
            
            if act.isAdjustable:
                phys = act.getLength(actMirPos) - act.neutralLength  
                
                # Check to see if act is a ComplexActuator where a more correct phys can be
                # determined based on the actuator geometry
                # isinstance is supposed to be harmful. Ask Russell.                
                try:
                    # compute new phys if act is a ComplexActuator
                    phys = act.doActTrigRO2(phys, actMirPos)
                except AttributeError:
                    pass
                # phys units: um
            else:
                 phys = None # no change in actuator length
            physList.append(phys)        
        return numpy.asarray(physList, dtype=float)

    def mount2Orient(self, mount):
        """
        translated from: src/subr/mir/onemount2orient.for
        
        Input:
        mount[0:5] = mount position for 6 actuators or encoders. 
        
        Output:
        orient[0:5] = mirror orientation with 6 axes 6 item list:
        orient[0] = piston (um)
        orient[1:3] = x-y tilt (")
        orient[3:5] = x-y translation (um)
        orient[5] = z rotation (")    
        
        """
        actuatorList = self.backwardList
        
        mount = numpy.asarray(mount, dtype=float)

        # first compute physical errors
        actNum = len(actuatorList)
        maxOrientErr = numpy.array([1., 0.001, 0.001, 0.1, 0.1, 1.])
        # compute physical position at zero orientation
        orient  = numpy.zeros((6,))
        physAtZero = self._orient2Phys(orient, actuatorList)
        
        # find fixed link indices, they won't be used for minimization
        fixLinkInd = ~numpy.isnan(physAtZero)
        
        # compute physical position at perturbed orientation
        # sum the square of errors
        # orient is zeros except one axis on each iteration?
        
        # diag matrix for extracting rows with a single non-zero element
        orient = numpy.diag(maxOrientErr,0) 
        
        maxPhysErr = numpy.zeros(actNum)
        for pert in orient:
            phys = self._orient2Phys(pert, actuatorList)
            maxPhysErr = maxPhysErr + (phys - physAtZero) ** 2    #sum over perturbations
        # if maxPhysErr <= something, quit?
        # 1 / error^2
        # remove fixedLink from calculation
        g_physMult = 1. / maxPhysErr[fixLinkInd]
        # g_physMult = numpy.nan_to_num(g_physMult) # Fixed links return phys = None
        g_phys = []
        for ind, act in enumerate(actuatorList):
            if act.isAdjustable:
                g_phys.append(act.lengthFromMount(mount[ind]))
        #initial guess ("home")
        initOrient = numpy.zeros(6)
        fitTol = 1e-8
        maxIter = 10000
        orient = scipy.optimize.fmin_powell(self._mount2Orient_func, initOrient, 
                                    args=(g_phys, g_physMult), 
                                    maxiter=maxIter, ftol = fitTol) 
                                    
        # take Z rot fixed links into account when computing orientation
        # set initial Z guess to 0
        for act in actuatorList:        
            if act.isAdjustable ==False:
            	# we have a fixed link
            	print 'adjusting Z rot'
            	fitTol = 1e-8
            	maxIter = 10000
            	zRot = scipy.optimize.fmin_powell(self._zRotMount2Orient_func, 0., 
                                            		args=(orient, act),
                                            		maxiter=maxIter, ftol=fitTol)
            	# put the freshly computed zRot back into orient
            	orient[5] = zRot
        return orient
        
    def orient2Mount(self, orient):
        """
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        orient[0:5] = mirror orientation with 6 axes 6 item list:
        orient[0] = piston (um)
        orient[1:3] = x-y tilt (")
        orient[3:5] = x-y translation (um)
        orient[5] = z rotation (")    

        Output: 
        mount[0:n-1] = mount position for n actuators
        """                            
        actuatorList = self.forwardList
        physList = self._orient2Phys(orient, actuatorList)
        mountList = [] 
        

        for ind, act in enumerate(actuatorList):
            if act.isAdjustable:
                mount = act.mountFromLength(physList[ind])
                if act.mountInRange(mount) != 1:
                    raise RuntimeError('mount=%f, out of range for actuator %i' % (mount,counter))
                mountList.append(mount)
            else:
                 mountList.append(None) # fixed link, no mount change
        return numpy.asarray(mountList, dtype=float)

class DirectMirrorEnc(DirectMirror):
    """
    A Direct Mirror Object with Encoders. All actuators connected directly to mirror.
    """
    
    def __init__(self, actuatorList, encoderList):
        """
        inputs: 
        - actuatorList: a list of 6 actuator objects
        - encoderList: a list of 6 encoders corresponding to actuators. If there is no
                        corresponding encoder for the actuator, enter None.
                        len(actuatorList) == len(encoderList)
        """
        DirectMirror.__init__(self, actuatorList = actuatorList)
        if len(encoderList) != 6:
            raise RuntimeError("encoderList=%s must be 6 elements, insert 'None' if \n\
                                there is a 'missing' encoder" % len(encoderList))
        
        # populate backwardList with encoders, using actuators in place of 'None' encoder.
        self.backwardList = []
        for ind, enc in enumerate(encoderList):
            if enc == None:
                self.backwardList.append(actuatorList[ind])
            else:
                self.backwardList.append(enc)
        
class TipTransMirror(DirectMirror):
    """
    Tip-Trans Mirror. Translate by tipping a central linear bearing. orient2phys method
    is different, other methods are same as DirectMirror obj.
    """
    def __init__(self, actuatorList, ctrMirZ, ctrBaseZ):
        DirectMirror.__init__(self, actuatorList)
        self.ctrMirZ = ctrMirZ
        self.ctrBaseZ = ctrBaseZ
        
    def _rotEqPol(self, inVec, eqAng, polAng):
        """
        This method was translated from src/subr/cnv/roteqpol.for.
        Rotates a 3-vector described by equatorial and polar angles, as follows:
        The plane of rotation contains the z axis and a line in the x-y plane
        at angle eqAng from the x axis towards y. The amount of rotation is
        angle polAng from the z axis towards the line in the x-y plane.
        
        inputs:
        - inVec: a 3 vector to be transformed
        - eqAng: equatorial rotation angle (radians) of line in x-y plane (from x to y).
                 This plane of rotation includes this line and z
        - polAng: polar rotation angle (radians) from z axis to the line in the x-y plane
        
        output:
        - outVec: a 3 vector that has been transformed by a rotation in eq and pol.
        """ 
        sinEq = math.sin(eqAng) 
        cosEq = math.cos(eqAng)
        sinPol = math.sin(polAng)
        cosPol = math.cos(polAng)

        rotMat = numpy.array([ [ sinEq**2 + (cosEq**2 * cosPol), 
                                  -sinEq * cosEq * (1 - cosPol),
                                                 cosEq * sinPol ],
                                                
                               [  -sinEq * cosEq * (1 - cosPol),
                                 cosEq**2 + (sinEq**2 * cosPol),
                                                 sinEq * sinPol ],
                                                
                               [                -cosEq * sinPol,
                                                -sinEq * sinPol,
                                                         cosPol ]  ])  
        outVec = numpy.dot(rotMat, inVec)
        return outVec
                                
    def _orient2Phys(self, orient, actuatorList):
        """
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
        
        Input:
        orient[0:5] = mirror orientation with 6 axes 6 item list:
        orient[0] = piston (um)
        orient[1:3] = x-y tilt (")
        orient[3:5] = x-y translation (um)
        orient[5] = z rotation (")     
        actuatorList = 6 element list of LinearActuator or Encoder objects.

        Output: 
        physList[0:n-1] =  delta length for n actuators, is n=6 necessary?
        """
        # compute values in mm and degrees
        pist    = orient[0] * MMPerMicron
        tiltX   = orient[1] * RadPerArcSec
        tiltY   = orient[2] * RadPerArcSec
        transX  = orient[3] * MMPerMicron
        transY  = orient[4] * MMPerMicron
        rotZ    = orient[5] * RadPerArcSec
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
        
        # create single vector rotation matrix, rotate by X first, Y second
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

        
        rotMatXY = numpy.dot(rotMatY, rotMatX) # this is right
        ctrMirPos = numpy.dot(rotMatXY, ctrUnrot)
        
        # Now apply translation to produce the final position.
        ctrMirPos = ctrMirPos + numpy.array([transX, transY, pist])
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
        sinEq = math.sin(eqAng) 
        cosEq = math.cos(eqAng)
        sinPol = math.sin(polAng)
        cosPol = math.cos(polAng)

        rotMatEq = numpy.array([ [ sinEq**2 + (cosEq**2 * cosPol), 
                                  -sinEq * cosEq * (1 - cosPol),
                                                 cosEq * sinPol ],
                                                
                               [  -sinEq * cosEq * (1 - cosPol),
                                 cosEq**2 + (sinEq**2 * cosPol),
                                                 sinEq * sinPol ],
                                                
                               [                -cosEq * sinPol,
                                                -sinEq * sinPol,
                                                         cosPol ]  ])
        physList = []
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
                    phys = act.doActTrigRO2(phys, actMirPos)
                except AttributeError:
                    pass
                        
                phys = act.getLength(actMirPos) - act.neutralLength 
   
                # phys units: um
            else:
                phys = None  # no change in actuator length
            physList.append(phys)
        return numpy.asarray(physList, dtype=float)