"""Mirror

The units for orientation are mm and radians (not user-friendly units, but best for computation).
"""
import collections
import itertools
import numpy
import scipy.optimize  
import math
# numpy.seterr(all='raise')
import link

# Orientation of mirror; units are mm and radians
Orientation = collections.namedtuple("Orientation", ["piston", "tiltX", "tiltY", "transX", "transY", "rotZ"])

# def userOrient(orient):
#     """Return orientation in user-friendly units
#     """
#     return 

class MirrorBase(object):
    def __init__(self, actuatorList, fixedLinkList, encoderList=None):
        """Construct a MirrorBase

        Inputs:
        - actuatorList: List of actuators that support the mirror.
        - fixedLinkList: List of fixed-length links that support the mirror
        - encoderList:  Encoders associated with actuators. None if there are no encoders,
                        else a list of items: each an encoder or None if the associated
                        actuator has no encoder.
        
        Two configurations are supported:
        - control piston, tip and tilt only (no translation or z rotation)
        - control piston, tip, tilt and translation (no z rotation)
        """
        if len(actuatorList) + len(fixedLinkList) != 6:
            raise RuntimeError("Need exactly 6 actuators + fixed-length links; %s + %s supplied" % \
                 (len(actuatorList), len(fixedLinkList)))
        self.actuatorList = actuatorList
        self.fixedLinkList = fixedLinkList
        self.hasEncoders = False
        if not encoderList:
            self.encoderList = actuatorList
        else:
            if len(encoderList) != len(actuatorList):
                raise RuntimeError("encoderList must contain %s encoders; has %s" % \
                    (len(actuatorList), len(encoderList)))
        
            # Populate encoderList with encoders, using actuators in place of 'None' encoder.
            self.encoderList = []
            for enc, act in itertools.izip(encoderList, actuatorList):
                if enc == None:
                    self.encoderList.append(act)
                else:
                    self.encoderList.append(enc)
                    self.hasEncoders = True

    def orientFromEncoderMount(self, mount):
        """Compute mirror orientation from encoder mount lengths
        
        Inputs:
        - mount: encoder mount positions; one per encoder or actuator if the actuator has no associated encoder
        """
        if len(mount) != len(self.encoderList):
            raise RuntimeError("Need %s values; got %s" % (len(self.encoderList), len(mount)))
        return self._orientFromMount(mount, self.encoderList)
    
    def orientFromActuatorMount(self, mount):
        """Compute mirror orientation from actuator mount lengths
        
        Inputs:
        - mount: encoder mount positions; one per encoder
        """
        if len(mount) != len(self.actuatorList):
            raise RuntimeError("Need %s values; got %s" % (len(self.actuatorList), len(mount)))
        return self._orientFromMount(mount, self.actuatorList)
    
    def actuatorMountFromOrient(self, orient):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - orient: an Orientation or collection of 6 items in the same order
        """
        return self._mountFromOrient(Orientation(*orient), self.actuatorList)
    
    def encoderMountFromOrient(self, orient):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - orient: an Orientation or collection of 6 items in the same order
        """
        return self._mountFromOrient(Orientation(*orient), self.encoderList)
    
    def _mountFromOrient(self, orient, linkList):
        """Compute link mount length from orientation
        """
        phys = self._physFromOrient(orient, linkList)
        return [link.mountFromPhys(p) for p, link in itertools.izip(phys, linkList)]
    
    def _physFromOrient(self, orient, linkList):
        """Compute physical link length from orientation.
        
        Subclasses must override
        """
        raise NotImplementedError()
    
    def _orientFromMount(self, mount, linkList):
        """Compute orientation from link mount length
        """
        phys = [link.physFromMount(mt) for mt, link in itertools.izip(mount, linkList)]
        return self._orientFromPhys(phys, linkList)
    
    def _orientFromPhys(self, phys, linkList):
        """Compute mirror orientation give the physical position of each actuator. Uses fitting.
        
        Input:
        - phys: physical position of each actuator or encoder; the position for fixed links will be ignored
        - linkList: list of actuators or encoders (not fixed links!)
       
        Output:
        - orient: mirror orientation (an Orientation)
        """
        numLinks = len(linkList)        

        # Compute physical errors
        maxOrientErr = Orientation(0.0001, 5e-8, 5e-8, 0.0001, 0.0001, 5e-7)
        
        # Compute delta physical length at perturbed orientations
        # Sum the square of errors
        # Orient is zeros except one axis on each iteration.
        
        # diagonal matrix for easily extracting orientations with a single non-zero element
        orientSet = numpy.diag(maxOrientErr, 0)
        
        maxPhysErrSq = numpy.zeros(numLinks)
        for pertOrient in orientSet:
            pertOrient = Orientation(*pertOrient)
            pertPhys = self._physFromOrient(pertOrient, linkList)
            maxPhysErrSq += pertPhys**2   
        physMult = 1.0 / maxPhysErrSq

        # initial guess ("home")
        initOrient = numpy.zeros(6)
        fitTol = 1e-8
        maxIter = 10000
        phys = numpy.asarray(phys, dtype=float) # is this necessary?
        orient = scipy.optimize.fmin_powell(
            self._orientFromPhysErr,
            initOrient, 
            args = (phys, physMult, linkList), 
            maxiter = maxIter,
            ftol = fitTol,
        )
                                    
        return Orientation(*orient)

    def _orient2RotTransMats(self, orient):
        """
        This function computes the rotation matrices and translation
        offsets from a given orientation. The outputs are used to
        transform the cartesian coordinates of actuators based on the
        orientation. 
        
        Input:
        - orient:  an Orientation

        Outputs: 
        - rotMat:  3x3 rotation matrix
        - offsets: 3x1 (x,y,z) offset vector
        """
        orient = Orientation(*orient) # needed for fitting orientFromPhys
        cosX = math.cos(orient.tiltX)
        sinX = math.sin(orient.tiltX)
        cosY = math.cos(orient.tiltY)
        sinY = math.sin(orient.tiltY)
        rotMatX = numpy.array([ [1,    0,     0],
                                [0, cosX, -sinX],
                                [0, sinX,  cosX] ])
                                
        rotMatY = numpy.array([ [cosY,  0, sinY],
                                [0,     1,    0],
                                [-sinY, 0, cosY] ])

        # Create single vector rotation matrix, rotate by X first, Y second
        rotMat = numpy.dot(rotMatY, rotMatX)

        sinZ = math.sin(orient.rotZ)
        cosZ = math.cos(orient.rotZ)
        rotMatZ = numpy.array([ [cosZ, -sinZ, 0],
                                [sinZ,  cosZ, 0],
                                [   0,     0, 1] ])
    
        rotMat = numpy.dot(rotMat, rotMatZ)

        offsets = numpy.array([orient.transX, orient.transY, orient.piston])
        
        return rotMat, offsets

        
class DirectMirror(MirrorBase):
    """A mirror supported by 6 actuators or fixed links connected directly to the mirror.
    """
    def __init__(self, actuatorList, encoderList=None):
        """
        Inputs:
        - actuatorList: List of the 6 actuators actuators and fixed links that support the mirror.
        - encoderList:  Encoders associated with actuators. None if there are no encoders,
                        else a list of 6 items: each an encoder or None if the associated
                        actuator has no encoder.
        
        Two configurations are supported:
        - control piston, tip and tilt only (no translation or z rotation)
        - control piston, tip, tilt and translation (no z rotation)
        """
        MirrorBase.__init__(self, actuatorList, encoderList)
              
    def _orientFromPhysErr(self, orient, phys, physMult, linkList):
        """Function to minimize while computing _orientFromPhys

        Inputs: 
        - orient: an Orientation
        - phys:  list of physical actuator lengths
        - physMult: list of multipliers for computing errors
        - linkList: list of adjustable actuators
        
        Output:
        - 1 + sum(physMult * physErrSq**2)
        """       
        physFromOrient = self._physFromOrient(orient, linkList)
        physErrSq = (physFromOrient - phys)**2
        return 1 + numpy.sum(physMult * physErrSq)
        
    def _physFromOrient(self, orient, linkList):
        """Compute desired physical position of actuators or encoders given mirror orientation
        
        Input:
        - orient: mirror orientation (an Orientation)
        - linkList: list of actuators or encoders

        Output: 
        - physList: physical length of each actuator (mm), measured from the neutral position.
                    FixedLengthLink links always return a phys of 0 because they cannot change length.
        """

        # Get rotation matrices and offsets.
        rotMat, offsets = self._orient2RotTransMats(orient)
        
		# list of acutator outputs in physical length (um), with neutral as the zero point.
        physList = []   
        for act in linkList:
            desMirPos = numpy.dot(rotMat, act.mirPos)
            desMirPos = desMirPos + offsets
            
            phys = act.physFromMirPos(desMirPos)
            physList.append(phys)        
        return numpy.asarray(physList, dtype=float)

    def _orientFromPhys(self, phys, linkList):
        """Compute mirror orientation give the physical position of each actuator. Uses fitting.
        
        Input:
        - phys: physical position of each actuator or encoder; the position for fixed links will be ignored
        - linkList: list of actuators or encoders (not fixed links!)
       
        Output:
        - orient: mirror orientation (an Orientation)
        """
        numLinks = len(linkList)        

        # Compute physical errors
        maxOrientErr = Orientation(0.0001, 5e-8, 5e-8, 0.0001, 0.0001, 5e-7)
        
        # Compute delta physical length at perturbed orientations
        # Sum the square of errors
        # Orient is zeros except one axis on each iteration.
        
        # diagonal matrix for easily extracting orientations with a single non-zero element
        orientSet = numpy.diag(maxOrientErr, 0)
        
        maxPhysErrSq = numpy.zeros(numLinks)
        for pertOrient in orientSet:
            pertOrient = Orientation(*pertOrient)
            pertPhys = self._physFromOrient(pertOrient, linkList)
            maxPhysErrSq += pertPhys**2   
        physMult = 1.0 / maxPhysErrSq

        # initial guess ("home")
        initOrient = numpy.zeros(6)
        fitTol = 1e-8
        maxIter = 10000
        phys = numpy.asarray(phys, dtype=float) # is this necessary?
        orient = scipy.optimize.fmin_powell(
            self._orientFromPhysErr,
            initOrient, 
            args = (phys, physMult, linkList), 
            maxiter = maxIter,
            ftol = fitTol,
        )
                                    
        return Orientation(*orient)

        
# class DirectMirrorZFix(DirectMirror):
#     """
#     A class of mirror with a FixedLengthLink constraining Z rotation. The only
#     added feature of this object is that an extra optimization is used
#     to incorporate the residual z rotation induced from the fixed link
#     at a given mirror orientation.
#     """
# 
#     def __init__(self, actuatorList, encoderList = None, rotZConst = bool(True)):
#         DirectMirror.__init__(self, actuatorList = actuatorList, encoderList = encoderList,
#                                rotZConst = rotZConst)
#         
#     def _zRotMin_func(self, Z, orient, act):
#         """
#         This method is iterated over to determine the best Z rotation solution
#         when there is a fixedLink constraining Z rotation. We minimize:
#         (measuredLinkLength - knownLinkLength)^2, there should only be one correct answer.
#         In reality this answer needs to be zero.
#    
#         Inputs:
#         - Z:      z rotation angle in ArcSec. Parameter to be optimized.
#         - orient: 6 axis orientation with Z rotation set to zero, this is 
#         			the output of mount2Orient.         
#         - act: 	  The FixedLengthLink, with a length that is conserved.
#         
#         Outputs: 1 + (measuredLinkLength - knownLinkLength)^2, thus the minimum
#         		   value we seek is 1.
#         """
# 
#         orient[5] = Z
#         rotMat, offsets = self._orient2RotTransMats(orient)
#         actUnrot = act.mirPos
#         
#         # rotate 3-vector
#         actMirPos = numpy.dot(rotMat, actUnrot)
#         
#         # apply all translations to compute final positions of
#         # the mirror end gimbals of the actuators
#         actMirPos = actMirPos + offsets
# 
#         deltaPhys = act.lengthFromMirPos(actMirPos) - act.neutralLength  
#         # in reality, deltaPhys = 0, because the measured distance between the fixed link's
#         # mirPos and basePos should always equal the neutralLength. We are using this fact to
#         # minimize the difference between measured length and expected length by varying Z
#         # to find the best solution.
#         return 1 + deltaPhys**2        
# 
#     def _computeZRot(self, orient):
#         """
#         For a given oriention, find the correct Z rotation value taking into account the FixedLengthLink
#         constraining the Z rotation.
#         """
#         # Take Z rotation fixed link into account when computing orientation.
#         # Although this mirror is constrained in Z rotation, we will turn the constraint off
#         # to compute the small amount of rotation that is induced when the mirror orientation
#         # changes.
#         self.rotZConst = False
# 
#         # Get fixed link constraining Z rotation, this is the only one that isn't adjustable
#         actuatorList = self.encoderList
#         for act in actuatorList:
#             if act.isAdjustable == False:               
# 				fitTol = 1e-8
# 				maxIter = 10000
# 				zRot = scipy.optimize.fmin_powell(self._zRotMin_func, 0., 
# 													args=(orient, act),
# 													maxiter=maxIter, ftol=fitTol)
# 				# Put the freshly computed zRot back into orient
# 				orient[5] = zRot
#         
#         # Turn Z constraint back on.
#         self.rotZConst = True
#         return orient
#     
#     def mount2Orient(self, mount):
#     	"""
#     	Most of the work is done in _mount2Orient, which will return with a z rotation of 0.
#     	The actual z rotation induced by the FixedLengthLink is then computed using _computeZRot.
#     	
#         Input:
#         - mount[0:n]: mount position for n actuators or encoders. 
#        
#         Output:
#         - orient[0:5]:  mirror orientation with 6 axes 6 item list:
#         - orient[0]:    piston (um)
#         - orient[1:3]:  x-y tilt (")
#         - orient[3:5]:  x-y translation (um)
#         - orient[5]:    z rotation ("). Non-zero!
#  
#     	"""
#         orient = self._mount2Orient(mount)
#         orient = self._computeZRot(orient)
#         return orient
#         
# class TipTransMirror(DirectMirror):
#     """
#     Tip-Trans Mirror. Translate by tipping a central linear bearing.
#     orient2phys method is different, other methods are same as
#     DirectMirror obj.
#     """
#     def __init__(self, ctrMirZ, ctrBaseZ, actuatorList, encoderList = None):
#         DirectMirror.__init__(self, actuatorList, encoderList = encoderList)
#         self.ctrMirZ = ctrMirZ
#         self.ctrBaseZ = ctrBaseZ
#         
#     def _rotEqPolMat(self, eqAng, polAng):
#         """
#         This method was translated from src/subr/cnv/roteqpol.for.
#         Defines a matrix that rotates a 3-vector described by equatorial
#         and polar angles as follows: The plane of rotation contains the
#         z axis and a line in the x-y plane at angle eqAng from the x
#         axis towards y. The amount of rotation is angle polAng from the
#         z axis towards the line in the x-y plane.
#         
#         Inputs:
#         - eqAng:  Equatorial rotation angle (radians) of line in x-y plane (from x to y).
#                    This plane of rotation includes this line and z.
#         - polAng: Polar rotation angle (radians) from z axis to the line in the x-y plane.
#         
#         Output:
#         - rotMatEqPol: 3x3 rotation matrix
#         """ 
#         sinEq = math.sin(eqAng) 
#         cosEq = math.cos(eqAng)
#         sinPol = math.sin(polAng)
#         cosPol = math.cos(polAng)
# 
#         rotMatEqPol = numpy.array([ [ sinEq**2 + (cosEq**2 * cosPol), 
#                                   -sinEq * cosEq * (1 - cosPol),
#                                                  cosEq * sinPol ],
#                                                 
#                                [  -sinEq * cosEq * (1 - cosPol),
#                                  cosEq**2 + (sinEq**2 * cosPol),
#                                                  sinEq * sinPol ],
#                                                 
#                                [                -cosEq * sinPol,
#                                                 -sinEq * sinPol,
#                                                          cosPol ]  ])  
#         return rotMatEqPol
#                                 
#     def _physFromOrient(self, orient, actuatorList):
#         """Compute physical actuator or encoder length given orientation.
#         
#         Input:
#         - orient:  mirror orientation with 6 axes 6 item list:
#         - actuatorList: list of actuators or encoders
# 
#         Output: 
#         - physList[0:5]: delta length for actuators (um) measured from the neutral position.
#         				        FixedLengthLink actuators always return a deltaPhys of 0 because they
#         				        cannot change length.
# 
#         translated from: src/subr/mir/oneorient2mount.for
#         src/subr/mir/oneOrient2Phys.for does most of work
#         """
#         # starting from line 152 in oneOrient2Phys.for (special case = tiptransmir)
#         
#         # Determine ctrMirPos, the final position of the central linear
#         # bearing mirror gimbal. This gimbal is fixed to the
#         # mirror (at ctrMirZ from the vertex) and slides along
#         # the central linear bearing.
#         # First rotate the gimbal position about the vertex
#         # (do this before translation so the vertex is at 0,0,0)
#         
#         ctrMirZ = self.ctrMirZ
#         ctrBaseZ = self.ctrBaseZ
#         ctrUnrot = numpy.zeros(3)
#         ctrUnrot[2] = ctrMirZ
#         
# 
#         rotMat, offsets = self._orient2RotTransMats(orient)
#         ctrMirPos = numpy.dot(rotMat, ctrUnrot)
#         
#         # Now apply translation to produce the final position.
#         ctrMirPos = ctrMirPos + offsets
#         # Determine ctrBasePos, the position of the central linear
#         # bearing base gimbal. This gimbal attaches the central linear
#         # bearing to the frame. Its position is fixed; it does not
#         # change as the mirror moves around.
#         ctrBasePos = numpy.zeros(3)
#         ctrBasePos[2] = ctrBaseZ
#         # Determine the vector from base gimbal to mirror gimbal.
#         ctrExtent = ctrMirPos - ctrBasePos
#         # Determine the new positions of the transverse actuators
#         # at the transverse gimbal end. This gimbal sets the tilt
#         # of the central linear bearing and is at a fixed length
#         # from the base gimbal. The home position of this end of the
#         # transverse actuators is actMirPos (despite the name).
#         #
#         # To do this, rotate the home position about the central linear
#         # bearing base pivot (ctrBasePos) by the amount the bearing tips.
#         # Note: rotation occurs in a plane defined by the motion
#         # of the central linear bearing, and is by the amount
#         # the bearing tips, so this is an "equatorial/polar" rotation
#         # (solved by nv_RotEqPol). I think. Close enough, anyway.
#         # First compute the equatorial and polar angle.
#         ctrLenXY = math.sqrt(numpy.sum(ctrExtent[0:2] ** 2))
#         if ctrExtent[2] > 0.:
#             eqAng = math.atan2(ctrExtent[1], ctrExtent[0])
#         else:
#             eqAng = math.atan2(-ctrExtent[1], -ctrExtent[0])
#         polAng = math.atan2(ctrLenXY, numpy.abs(ctrExtent[2]))
#         # compute home position of transverse actuators
#         # at transverse gimbal end, with respect to the base gimbal
#         
#         # make eq-pol rotation matrix
#         rotMatEq = self._rotEqPolMat(eqAng, polAng)
#        
#         physList = []
#         for act in actuatorList:
#             actUnrot = numpy.asarray(act.mirPos, dtype=float) - ctrBasePos
#             # rotate this about the base pivot to compute the actual position
#             # of the transverse actuators at the transverse gimbal end,
#             # with respect to the base pivot
#             actMirPos = numpy.dot(rotMatEq, actUnrot)
#             # compute the position of the transverse actuators at the
#             # transverse gimbal end to the standard reference frame
#             # (which is with respect to home position of mirror vertex).
#             actMirPos = actMirPos + ctrBasePos
#             if act.isAdjustable:
#                 try:
#                     # compute new phys if act is a AdjBaseActuator
#                     deltaPhys = act.physFromMirPosRO(deltaPhys, actMirPos)
#                 except AttributeError:
#                     pass                        
#                 deltaPhys = act.lengthFromMirPos(actMirPos) - act.neutralLength    
#                 # phys units: um
#             else:
#                 deltaPhys = 0.  # no change in actuator length (FixedLengthLink)
#             physList.append(deltaPhys)
#         return numpy.asarray(physList, dtype=float)
