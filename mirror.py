"""Mirror

The units for orientation are mm and radians (not user-friendly units, but best for computation).
"""
import collections
import itertools
import copy
import numpy
import scipy.optimize  
import math
# numpy.seterr(all='raise')
import link

fitTol = 1e-8
maxIter = 10000
# Orientation of mirror; units are mm and radians
Orientation = collections.namedtuple("Orientation", ["piston", "tiltX", "tiltY", "transX", "transY", "rotZ"])

# def userOrient(orient):
#     """Return orientation in user-friendly units
#     """
#     return 

class MirrorBase(object):
    def __init__(self, actuatorList, fixedLinkList, encoderList):
        """Construct a MirrorBase

        Inputs:
        - actuatorList: List of actuators that support the mirror.
        - fixedLinkList: List of fixed-length links that support the mirror
        - encoderList:  Encoders associated with actuators. None if there are no encoders,
                        else a list of items: each an encoder or None if the associated
                        actuator has no encoder.
        - fixAxes: List of orientation axes that are fixed. Integer values [0:5].
                        Must be specified if fixed length links are present.
        
        Two configurations are supported:
        - control piston, tip and tilt only (no translation or z rotation)
        - control piston, tip, tilt and translation (no z rotation)
        """
        if len(actuatorList) + len(fixedLinkList) != 6:
            raise RuntimeError("Need exactly 6 actuators + fixed-length links; %s + %s supplied" % \
                 (len(actuatorList), len(fixedLinkList)))
        self.actuatorList = actuatorList
        
        if len(fixedLinkList) in [0, 1, 3] == False:
            raise RuntimeError("Any mirror may only have 0, 1, or 3 fixed links. %s were supplied"\
                                 % (len(fixedLinkList)))
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
    
    def actuatorMountFromOrient(self, userOrient):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - userOrient: an Orientation or collection of 6 items or fewer in the same order. Keywords
        are also accepted, undefined fields will be silently replaced as zeros.
        """
        orient = self._fullOrient(userOrient)
        return self._mountFromOrient(Orientation(*orient), self.actuatorList)
    
    def encoderMountFromOrient(self, userOrient):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - userOrient: an Orientation or collection of 6 items or fewer in the
        same order. Keywords are also accepted, undefined fields will be
        silently replaced as zeros.
        """
        orient = self._fullOrient(userOrient)
        return self._mountFromOrient(Orientation(*orient), self.encoderList)
        
    def _fullOrient(self, userOrient):
        """Takes user supplied orientation values and constructs a fully defined
        Orientation to be used throughout. This routine will also optimize
        orientations in the case of fixedLinkList length links.

        Input:
        - userOrient: list of orientation values, or keword/value pairs.
        Keywords correspond to those defined in the Orientation object.
        Non-specified orientation axes are replaced as zeros. 
        
        Output:
        - orient: a fully defined (and sometimes adjusted) orientation
        """
        if len(userOrient) in [0, 1, 3, 5] == False:
            raise RuntimeError('Input orientation must be a list of numbers of \n\
                                length 0, 1, 3, or 5. Actual length %s' % (len(userOrient)))
        orient = numpy.zeros(6)
        for axis, val in enumerate(userOrient):
            orient[axis] = val
        
        # Get adjusted orient. Takes fixed axes into account and returns a
        # (new) orient.
        
        if len(self.fixedLinkList) > 0:
            # assume only axis 5 (z rotation) is fixed for now to test
            if len(self.fixedLinkList) == 1:
                # one fixed link means z anti-rotation link
                fixAxes = [5] 
            if len(self.fixedLinkList) == 3:
                # three fixed links means z rotation, transX, and transY are constrained
                fixAxes = [3,4,5]
            orient = numpy.asarray(orient, dtype=float)
            orient = self._adjustOrient(orient, fixAxes)
        
        return orient
            
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

    def _physMult(self, linkList):
        """Determine multipliers to be used in minimization of orientation errors, so we
        minimize a dimensionless quantity.
        
        Input:
        - linkList
        
        Output:
        - physMult: error multiplyer for each axis.
        """
        
        maxOrientErr = Orientation(0.0001, 5e-8, 5e-8, 0.0001, 0.0001, 5e-7)
        
        # Compute delta physical length at perturbed orientations
        # Sum the square of errors
        # Orient is zeros except one axis on each iteration.
        
        # diagonal matrix for easily extracting orientations with a single non-zero element
        orientSet = numpy.diag(maxOrientErr, 0)
        maxPhysErrSq = numpy.zeros(len(linkList))
        for pertOrient in orientSet:
            pertOrient = Orientation(*pertOrient)
            pertPhys = self._physFromOrient(pertOrient, linkList)    
            maxPhysErrSq += pertPhys**2   
        physMult = 1.0 / maxPhysErrSq
        
        return physMult
    
    def _orientFromPhys(self, phys, linkList):
        """Compute mirror orientation give the physical position of each actuator. Uses fitting.
        
        Input:
        - phys: physical position of each actuator or encoder; the position for fixed links will be ignored
        - linkList: list of actuators or encoders (not fixed links!)
       
        Output:
        - orient: mirror orientation (an Orientation)
        """
        givPhys = phys
        # we want to compute orient using fixed link constraints
        linkListFull = linkList + self.fixedLinkList 
        # now add a phys of zero for fixed links.
        givPhys.extend(numpy.zeros(len(self.fixedLinkList)))
        numLinks = len(linkListFull)
        # Compute physical errors
        physMult = self._physMult(linkListFull)

        # initial guess ("home")
        
        
        initOrient = numpy.zeros(6)
        orient = scipy.optimize.fmin_powell(
            self._minOrientErr,
            initOrient, 
            args = (givPhys, physMult, linkListFull), 
            maxiter = maxIter,
            ftol = fitTol,
            disp = False
        )
  
        return Orientation(*orient)        

    def _adjustOrient(self, orient, fixAxes):
        """This method adjusts the 'commanded' orientation based on the presence of fixed length
        links. 
        
        For example, a link constraining z rotation will induce a small amount of z rotation
        upon movement of the mirror. We want to solve for this induced z rotation and plug it back
        into the 'commanded' orientation.  This method is generalized for any fixed axis.
        
        Input:
        - orient: desired 6 axis Orientation
        - fixAxes: list of integers ranging from 0 to 5. Each element 
                    corresponds to an axis in orient that is fixed.
                    
        Output:
        - orientOut: adjusted 6 axis Orientation.
        """
        linkList = self.fixedLinkList
        physMult = self._physMult(linkList)
        # phys lengths should be zero, since we're dealing with fixed length links
        givPhys = numpy.zeros(len(fixAxes))
        # only searching for solutions for the fixed axes
        initOrient = numpy.zeros(len(fixAxes))
        orientFix = scipy.optimize.fmin_powell(
                        self._minOrientErr, initOrient,
                        args = (givPhys, physMult, linkList, fixAxes, orient), 
                        maxiter = maxIter,
                        ftol = fitTol,
                        disp = False
                        )
        orient[fixAxes] = orientFix
        return orient
        
    def _minOrientErr(self, minOrient, givPhys, physMult, linkList, fitAxes=None, fullOrient=None):
        """This function minimizes the error between a computed phys (from an orientation) and the
        given phys (givPhys).  If fixAxes is defined, only orientations for those axes are found. 
        In this case a fullOrient(6) is needed to define the orientation values that are held 
        constant (the fitter won't touch these).
        
        Inputs: 
        - minOrient: an orientation to be solved for. If it is less than 6 elements, then fitAxes
                        needs to be defined.
        - givPhys:  list of physical link lengths, must be same length as linkList
        - physMult: list of multipliers for computing errors
        - linkList: list of links
        - fitAxes:  axes to solve for if minOrient is less than 6 elements. Must be same size
                    as minOrient.
        - fullOrient: 6 item collection. This is a 'constant' orientation, minOrient is inserted
                        into specific axes of fullOrient defined by fitAxes. This way we can define
                        a minimization for an arbitrary amount of orientation axes while leaving 
                        the constant axes untouched by the fitter.
        
        Output:
        - 1 + sum(physMult * physErrSq**2)
        """
        # I have omitted error checking for speed since this is iterated upon
        if len(minOrient) < 6:
            # combine the fixed axis values (recomputed each loop) with desOrient
            desOrient = numpy.asarray(fullOrient, dtype=float)
            minOrient = numpy.asarray(minOrient, dtype=float)  # numpy needed for indexing
            # this way only the fitAxes are being varied by the fitter 
            desOrient[fitAxes] = minOrient
        else:
            desOrient = minOrient
            
        physList = self._physFromOrient(desOrient, linkList)

        # note givPhys should always = 0 for fixed length links
        physErrSq = (physList - givPhys)**2
        return 1 + numpy.sum(physMult * physErrSq)
                        
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
    def __init__(self, actuatorList, fixedLinkList, encoderList = None):
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
        MirrorBase.__init__(self, actuatorList, fixedLinkList, encoderList)

    def _physFromOrient(self, orient, linkList):
        """Compute desired physical position of actuators, encoders or fixed
        length links given mirror orientation If a fixed length link is included
        in linkList, the resulting phys value is probably unobtainable. We use
        this fact in minimization routines, to solve for correct orientations
        given the fact that in reality phys = 0 for fixed length links.
        
        Input:
        - orient: mirror orientation (an Orientation)
        - linkList: list of actuators, encoders, or fixed length links

        Output: 
        - physList: computed physical distance to the mirror (mm), measured from the neutral 
        position. In reality, fixed lenght links must have a phys of 0. This method doesn't 
        return necessary 0 for fixed links.
                    
        """
        rotMat, offsets = self._orient2RotTransMats(orient)
        physList = []   
        for act in linkList:
            desMirPos = numpy.dot(rotMat, act.mirPos)
            desMirPos = desMirPos + offsets
            # compute phys in a way that works for fixed links
            physList.append(act.lengthFromMirPos(desMirPos) - act.neutralLength)
        physList = numpy.asarray(physList, dtype=float)
        return physList
                      
#     def _physFromOrient(self, orient, linkList):
#         """Compute desired physical position of actuators or encoders given mirror orientation
#         
#         Input:
#         - orient: mirror orientation (an Orientation)
#         - linkList: list of actuators or encoders
# 
#         Output: 
#         - physList: physical length of each actuator (mm), measured from the neutral position.
#                     FixedLengthLink links always return a phys of 0 because they cannot change length.
#         """
# 
# 
#         # Get rotation matrices and offsets.
#         rotMat, offsets = self._orient2RotTransMats(orient)
#         
# 		# list of acutator outputs in physical length (um), with neutral as the zero point.
#         physList = []   
#         for act in linkList:
#             desMirPos = numpy.dot(rotMat, act.mirPos)
#             desMirPos = desMirPos + offsets
#             
#             phys = act.physFromMirPos(desMirPos)
#             physList.append(phys)
#         return numpy.asarray(physList, dtype=float)

      
class TipTransMirror(MirrorBase):
    """
    Tip-Trans Mirror. Translate by tipping a central linear bearing.
    orient2phys method is different, other methods are same as
    DirectMirror obj.
    """
    def __init__(self, ctrMirZ, ctrBaseZ, actuatorList, fixedLinkList, encoderList = None):
        MirrorBase.__init__(self, actuatorList, fixedLinkList, encoderList)
        self.ctrMirZ = ctrMirZ
        self.ctrBaseZ = ctrBaseZ
        if len(self.fixedLinkList) != 1:
            raise RuntimeError('TipTrans Mirror needs one z anti-rotation fixed length link defined\
                                %s were supplied' % (len(self.fixedLinkList)))
        
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
                                       
    def _physFromOrient(self, orient, linkList):
        """Compute physical actuator, encoder, or fixed length length given orientation.
        
        Input:
        - orient:  mirror orientation with 6 axes 6 item list:
        - actuatorList: list of actuators or encoders

        Output: 
        - physList[0:5]: delta length for link length (um) measured from the neutral position.
        				        FixedLengthLink can return a non-zero length.

        notes: This can return an impossible length in the case of fixed length links. This feature
        is actually desired for the minimization routines.
        
        translated from: src/subr/mir/oneorient2mount.for
        src/subr/mir/oneOrient2Phys.for does most of work
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
        
        rotMat, offsets = self._orient2RotTransMats(orient)
        ctrMirPos = numpy.dot(rotMat, ctrUnrot)
        
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
       
        physList = []
        for ind, act in enumerate(linkList):
            if ind == 3 or ind == 4:  
                # these are special transverse actuators
                actUnrot = numpy.asarray(act.mirPos, dtype=float) - ctrBasePos
                # rotate this about the base pivot to compute the actual position
                # of the transverse actuators at the transverse gimbal end,
                # with respect to the base pivot
                desMirPos = numpy.dot(rotMatEq, actUnrot)
                # compute the position of the transverse actuators at the
                # transverse gimbal end to the standard reference frame
                # (which is with respect to home position of mirror vertex).
                desMirPos = desMirPos + ctrBasePos
                phys = (act.lengthFromMirPos(desMirPos) - act.neutralLength)
                physList.append(phys)
            else:
                desMirPos = numpy.dot(rotMat, act.mirPos)
                desMirPos = desMirPos + offsets
                phys = (act.lengthFromMirPos(desMirPos) - act.neutralLength)
                physList.append(phys)        
        return numpy.asarray(physList, dtype=float)

         


