"""Mirror

The units for orientation are mm and radians (not user-friendly units, but best for computation).
"""
import collections
import itertools
import math

import numpy
import scipy.optimize  

import link

numpy.seterr(all='raise')

_FitTol = 1e-8
_MaxIter = 10000
# Orientation of mirror; units are mm and radians
Orientation = collections.namedtuple("Orientation", ["piston", "tiltX", "tiltY", "transX", "transY", "rotZ"])
ZeroOrientation = Orientation(0, 0, 0, 0, 0, 0)

# def userOrient(orient):
#     """Return orientation in user-friendly units
#     """
#     return 

class MirrorBase(object):
    def __init__(self, actuatorList, fixedLinkList, encoderList, name=None):
        """Construct a MirrorBase

        Inputs:
        - actuatorList: List of actuators that support the mirror.
        - fixedLinkList: List of fixed-length links that support the mirror
        - encoderList:  Encoders associated with actuators. None if there are no encoders,
                        else a list of items: each an encoder or None if the associated
                        actuator has no encoder.
        - fixAxes: List of orientation axes that are fixed. Integer values [0:5].
                        Must be specified if fixed length links are present.
        - name: Mirror name, if you care to specify. Choose from 'Prim', 'Sec', or 'Tert'
                this is read by the actor and used to generate appriate keywords
        
        Two configurations are supported:
        - control piston, tip and tilt only (no translation or z rotation)
        - control piston, tip, tilt and translation (no z rotation)
        """
        self.name = name
        if len(actuatorList) + len(fixedLinkList) != 6:
            raise RuntimeError("Need exactly 6 actuators + fixed-length links; %s + %s supplied" % \
                 (len(actuatorList), len(fixedLinkList)))
        self.actuatorList = actuatorList
        
        # be sure to use a list for self._fixedAxes so it can be used to index numpy arrays
        if len(fixedLinkList) == 0:
            self._fixedAxes = []
        elif len(fixedLinkList) == 1:
            # one antirotation link: z rotation is constrained
            self._fixedAxes = [5]
        elif len(fixedLinkList) == 3:
            # x,y translation and z rotation are constrained
            self._fixedAxes = [3, 4, 5]
        else:            
            raise RuntimeError("Any mirror may only have 0, 1, or 3 fixed links. %s were supplied"\
                                 % (len(fixedLinkList)))
        self.fixedLinkList = fixedLinkList    

        self.hasEncoders = False
        if not encoderList:
            self.encoderList = actuatorList
        else:
            self.hasEncoders = True
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
                    
    
    @property
    def numAdjOrient(self):
        """Return the number of adjustable axes of orientation
        
        Warning: even if rotZ is technically adjustable it is never counted
        (the max returned value is always 5)
        """
        return min(5, 6 - len(self._fixedAxes))

    def orientFromEncoderMount(self, mount, initOrient=ZeroOrientation):
        """Compute mirror orientation from encoder mount lengths
        
        Inputs:
        - mount: encoder mount positions; one per encoder or actuator if the actuator has no associated encoder
        - initOrient: initial guess as to orientation; may be incomplete
        """
        if len(mount) != len(self.encoderList):
            raise RuntimeError("Need %s values; got %s" % (len(self.encoderList), len(mount)))
        return self._orientFromMount(mount, self.encoderList, initOrient)
    
    def orientFromActuatorMount(self, mount, initOrient = ZeroOrientation):
        """Compute mirror orientation from actuator mount lengths
        
        Inputs:
        - mount: encoder mount positions; one per encoder
        - initOrient: initial guess as to orientation; may be incomplete
        """
        if len(mount) != len(self.actuatorList):
            raise RuntimeError("Need %s values; got %s" % (len(self.actuatorList), len(mount)))
        return self._orientFromMount(mount, self.actuatorList, initOrient)
    
    def actuatorMountFromOrient(self, userOrient, return_adjOrient = False):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - userOrient: an Orientation or collection of 6 items or fewer in the same order.
            Missing items are treated as 0 or (for uncontrollable axes) properly constrained values.
        
        Output:
        - mountList: list of mount coords for actuator/encoders
        - adjOrient: adjusted orient based on mirror fixed links
        """
        adjOrient = self._fullOrient(userOrient)
        mountList = self._mountFromOrient(adjOrient, self.actuatorList)
        if return_adjOrient == True:
            return mountList, adjOrient
        else:
            return mountList
        
    def encoderMountFromOrient(self, userOrient, return_adjOrient = False):
        """Compute actuator mount lengths from orientation
        
        Inputs:
        - userOrient: an Orientation or collection of 6 items or fewer in the
        same order. Keywords are also accepted, undefined fields will be
        silently replaced as zeros.
        
        Output:
        - mountList: list of mount coords for actuator/encoders
        - adjOrient: adjusted orient based on mirror fixed links
        """
        adjOrient = self._fullOrient(userOrient)
        mountList = self._mountFromOrient(adjOrient, self.encoderList)
        if return_adjOrient == True:
            return mountList, adjOrient
        else:
            return mountList
        
    def _fullOrient(self, userOrient):
        """Compute fully specified orientation from a partially specified orientation.
        
        Input:
        - userOrient: list of orientation values: 0, 1, 3, 5 or 6 values. Missing values are treated as 0.

        Output:
        - orient: the full 6-axis orientation as an Orientation. Axes that cannot be controlled
            are set to their constrained value (which should be nearly 0 for a typical mirror).
        """
        if len(userOrient) not in set((0, 1, 3, 5, 6)):
            raise RuntimeError('Input orientation must be a list of numbers of \n\
                                length 0, 1, 3, or 5, 6. Actual length %s' % (len(userOrient)))

        # set all non-supplied orientations to 0.
        orient = numpy.zeros(6, dtype=float)
        orient[0:len(userOrient)] = userOrient

        # compute constrained axes of orientation
        if self._fixedAxes:
            linkList = self.fixedLinkList
            physMult = self._physMult(linkList)
            # phys lengths should be zero, since we're dealing with fixed length links
            givPhys = numpy.zeros(len(self._fixedAxes))
            # only searching for solutions for the fixed axes
            initOrient = numpy.zeros(len(self._fixedAxes))
            minOut = scipy.optimize.fmin(
                            self._minOrientErr, initOrient,
                            args = (givPhys, physMult, linkList, self._fixedAxes, orient), 
                            maxiter = _MaxIter,
                            ftol = _FitTol,
                            disp = False,
                            full_output = True
                            )
            # if (re)using fmin (dhill simplex) uncomment below                        
#             minOut = scipy.optimize.fmin(
#                             self._minOrientErr, minOut[0],
#                             args = (givPhys, physMult, linkList, self._fixedAxes, orient), 
#                             maxiter = _MaxIter,
#                             ftol = _FitTol,
#                             disp = False,
#                             full_output = True
#                             )
                            
            fitOrient = minOut[0]
            warnflag = minOut[-1]
            if warnflag == 2:
                raise RuntimeError('Too many iterations')
            if warnflag == 1:
                raise RuntimeError('Too many function calls')
            orient[self._fixedAxes] = fitOrient

        return Orientation(*orient)

    def _mountFromOrient(self, orient, linkList):
        """Compute link mount length from orientation
        """
        phys = self._physFromOrient(orient, linkList)
        mountList = []
        for p, link in itertools.izip(phys, linkList):
            mount = link.mountFromPhys(p)
            if link.minMount <= mount <= link.maxMount:
                mountList.append(mount)
            else:
                raise RuntimeError('Mount out of range!')
        
        return mountList
        
        #return [link.mountFromPhys(p) for p, link in itertools.izip(phys, linkList)]
    
    def _physFromOrient(self, orient, linkList):
        """Compute physical link length from orientation.
        
        Subclasses must override
        """
        raise NotImplementedError()
    
    def _orientFromMount(self, mount, linkList, initOrient=ZeroOrientation):
        """Compute orientation from link mount length

        Inputs:
        - mount: link mount length
        - linkList: list of Links
        - initOrient: initial guess as to orientation (may be incomplete)
        """
        if len(mount) != len(linkList):
            raise RuntimeError("len(mount)=%s != %s=len(linkList)" % (len(mount), len(linkList)))
        phys = [link.physFromMount(mt) for mt, link in itertools.izip(mount, linkList)]
        return self._orientFromPhys(phys, linkList, initOrient)    

    def _physMult(self, linkList):
        """Determine multiplier for each actuator to be used in minimization
        
        Computes the effect of a perturbing each axis of orientation by a given amount
        on each link actuator length. This is used to scale the weighting of physical
        length errors when fitting orientation from physical length.
        
        Input:
        - linkList: list of links
        
        Output:
        - physMult: error multiplier for each axis.
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
    
    def _orientFromPhys(self, phys, linkList, initOrient=ZeroOrientation):
        """Compute mirror orientation give the physical position of each actuator. Uses fitting.
        
        Input:
        - phys: physical position of each actuator or encoder; the position for fixed links will be ignored
        - linkList: list of actuators or encoders (not fixed links!)
        - initOrient: initial guess as to orientation; may be incomplete
       
        Output:
        - orient: mirror orientation (an Orientation)
        """
        givPhys = phys[:]
        # we want to compute orient using fixed link constraints
        linkListFull = linkList + self.fixedLinkList 
        # now add a phys of zero for fixed links.
        givPhys.extend(numpy.zeros(len(self.fixedLinkList)))
        numLinks = len(linkListFull)
        # Compute physical errors
        physMult = self._physMult(linkListFull)
        
        fullInitOrient = self._fullOrient(initOrient)

        minOut = scipy.optimize.fmin_powell(
            self._minOrientErr,
            fullInitOrient, 
            args = (givPhys, physMult, linkListFull), 
            maxiter = _MaxIter,
            ftol = _FitTol,
            disp = 0,
            full_output = 1
        )
        # if (re)using dhill simplex uncomment below       
#         minOut = scipy.optimize.fmin(
#             self._minOrientErr,
#             minOut[0], 
#             args = (givPhys, physMult, linkListFull), 
#             maxiter = _MaxIter,
#             ftol = _FitTol,
#             disp = 0,
#             full_output = 1
#         )
        orient = minOut[0]
        warnflag = minOut[-1]
        # print function value
        # print 'f val: ', minOut[1]
        if warnflag == 2:
            raise RuntimeError('Maximum num of iterations reached, computing mount-->orient')
        elif warnflag == 1:
            raise RuntimeError('Maximum num of func calls reached, computing mount-->orient')
        else:
            return Orientation(*orient)        
        
    def _minOrientErr(self, minOrient, givPhys, physMult, linkList, fitAxes=None, fullOrient=None):
        """Compute physical error given desired orientation and actual physical.
        
        If fixAxes is defined, only orientations for those axes are found. 
        In this case a fullOrient(6) is needed to define the orientation values that are held 
        constant (the fitter won't touch these).
        
        Inputs: 
        - minOrient: an orientation to be solved for. If it is less than 6 elements, then fitAxes
                        must to be specified and must be the same length as minOrient.
        - givPhys:  list of physical link lengths, must be same length as linkList
        - physMult: list of multipliers for computing errors
        - linkList: list of links
        - fitAxes:  a Python list of axes to solve for.
                        Ignored if minOrient has 6 elements, else must be the same size as minOrient.
        - fullOrient: 6 item collection. This is a 'constant' orientation, minOrient is inserted
                        into specific axes of fullOrient defined by fitAxes. This way we can define
                        a minimization for an arbitrary amount of orientation axes while leaving 
                        the constant axes untouched by the fitter.
        
        Output:
        - sum(physMult * physErrSq**2)
        
        @raise IndexError if fitAxes is not a Python list
        """
        # I have omitted error checking for speed since this is iterated upon
        if len(minOrient) < 6:
            # combine the fixed axis values (recomputed each loop) with desOrient
            # this way only the fitAxes are being varied by the fitter
            desOrient = numpy.array(fullOrient, dtype=float)
            desOrient[fitAxes] = minOrient
        else:
            desOrient = minOrient        
        physList = self._physFromOrient(desOrient, linkList)
        # note givPhys should always = 0 for fixed length links
        physErrSq = (physList - givPhys)**2
        return numpy.sum(physMult * physErrSq) # for chi squared physMult is 1 / variance
        

    def _orient2RotTransMats(self, orient):
        """Compute transformation matrices for mirror positions for a given orientation.
        
        Compute matrices used to transform cartesian points on the mirror from
        their location at zero orientation to their location at the specified orientation.
        
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
        # small angle versions        
#         cosX = 1 - (orient.tiltX)**2. / 2.
#         sinX = orient.tiltX
#         cosY = 1 - (orient.tiltY)**2. / 2.
#         sinY = orient.tiltY
        
        
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
        # small angle versions
#         sinZ = orient.rotZ #small ang approx
#         cosZ = 1 - (orient.rotZ)**2. / 2.  #small ang approx
        
        rotMatZ = numpy.array([ [cosZ, -sinZ, 0],
                                [sinZ,  cosZ, 0],
                                [   0,     0, 1] ])
    
        rotMat = numpy.dot(rotMat, rotMatZ)

        offsets = numpy.array([orient.transX, orient.transY, orient.piston])
        
        return rotMat, offsets
        
class DirectMirror(MirrorBase):
    """A mirror supported by 6 actuators or fixed links connected directly to the mirror.
    """
    def __init__(self, actuatorList, fixedLinkList, encoderList = None, name=None):
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
        MirrorBase.__init__(self, actuatorList, fixedLinkList, encoderList, name)

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
        for ind, act in enumerate(linkList):
            desMirPos = numpy.dot(rotMat, act.mirPos)
            desMirPos = desMirPos + offsets
            # compute phys in a way that works for fixed links
            physList.append(act.physFromMirPos(desMirPos))
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
#       # list of acutator outputs in physical length (um), with neutral as the zero point.
#         physList = []   
#         for act in linkList:
#             desMirPos = numpy.dot(rotMat, act.mirPos)
#             desMirPos = desMirPos + offsets
#             
#             phys = act.physFromMirPos(desMirPos)
#             physList.append(phys)
#         return numpy.asarray(physList, dtype=float)

      
class TipTransMirror(MirrorBase):
    """Tip-Trans Mirror.
    
    A mirror that rides on a central linear bearing;
    the bearing is tipped about a ball joint to translate the mirror.
    Actuators 0, 1, 2 are axial actuators attached to the mirror.
    Actuators 3, 4 are attached to the linear bearing to tip it.
    Actuator 5 is an antirotation link attached to the mirror.
    """
    def __init__(self, ctrMirZ, ctrBaseZ, actuatorList, fixedLinkList, encoderList = None, name=None):
        MirrorBase.__init__(self, actuatorList, fixedLinkList, encoderList, name)
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
        

        ctrUnrot = numpy.zeros(3)
        ctrUnrot[2] = self.ctrMirZ
        
        rotMat, offsets = self._orient2RotTransMats(orient)
        ctrMirPos = numpy.dot(rotMat, ctrUnrot)
        # Now apply translation to produce the final position.
        ctrMirPos = ctrMirPos + offsets
        # Determine ctrBasePos, the position of the central linear
        # bearing base gimbal. This gimbal attaches the central linear
        # bearing to the frame. Its position is fixed; it does not
        # change as the mirror moves around.
        ctrBasePos = numpy.zeros(3)
        ctrBasePos[2] = self.ctrBaseZ
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
        ctrLenXY = numpy.linalg.norm(ctrExtent[0:2])
        #if ctrExtent[2] > 0.:
        #    eqAng = math.atan2(ctrExtent[0], ctrExtent[1])
        #else:
        #    eqAng = math.atan2(-ctrExtent[0], -ctrExtent[1])
        eqAng = math.atan2(ctrExtent[0], ctrExtent[1])
        if eqAng < 0:
            eqAng += 2 * math.pi
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
                phys = (act.physFromMirPos(desMirPos))
                physList.append(phys)
            else:
                desMirPos = numpy.dot(rotMat, act.mirPos)
                desMirPos = desMirPos + offsets
                phys = (act.physFromMirPos(desMirPos))
                physList.append(phys)        
        return numpy.asarray(physList, dtype=float)
        
    def _physFromOrientCCS(self, orient, linkList):
        """Compute physical actuator, encoder, or fixed length length given orientation. 
        This version differs from _physFromOrient because I choose to work in spherical coords
        for the transverse actuators.
        
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
        
        # used for non-transverse actuators, since they are direct to the mirror
        rotMat, offsets = self._orient2RotTransMats(orient)
        
        # do something special for the transverse actuators        
        
        
        mirGimb = numpy.zeros(3)
        mirGimb[2] = self.ctrMirZ
        baseGimb = numpy.zeros(3)
        baseGimb[2] = self.ctrBaseZ 
        # apply desired xy translation and z piston
        desMirGimb = mirGimb + offsets
        # take the base gimbal to be origin
        desMirGimb = desMirGimb - baseGimb
        # get polar (from +Z) and equatorial (from +X, 0 - 2pi rad) angles
        eqAng = math.atan2(desMirGimb[0], desMirGimb[1])
        if eqAng < 0:
            # atan2 is defined from -pi:pi, for spherical coords we want 0:2*pi
            eqAng += 2 * math.pi
        polAng = math.acos(desMirGimb[2] / numpy.linalg.norm(desMirGimb))
        
        
        
       
        physList = []
        for ind, act in enumerate(linkList):
            if ind == 3 or ind == 4:  
                # these are special transverse actuators
                # again use baseGimb as origin for rotation
                actUnrot = act.mirPos - baseGimb
                # convert to polar coords (r, theta, phi)
                r = numpy.linalg.norm(actUnrot)
                theta = math.atan2(actUnrot[0], actUnrot[1])
                if theta < 0:
                    # theta must be defined from 0:2pi
                    theta += 2 * math.pi
                phi = math.acos(actUnrot[2] / r)
                
                actUnrotPol = numpy.asarray([r, theta, phi], dtype=float)
                # now apply eqAng and polAng offsets, but they are inverted because
                # we are now looking from the bottom
                desMirPosPol = actUnrotPol + numpy.asarray([0., eqAng, polAng], dtype=float)
                # now go back to cartesian coords. wow this is really a bad way to do this
                x = desMirPosPol[0] * math.cos(desMirPosPol[1]) * math.sin(desMirPosPol[2])
                y = desMirPosPol[0] * math.sin(desMirPosPol[1]) * math.sin(desMirPosPol[2])
                z = desMirPosPol[0] * math.cos(desMirPosPol[2])
                
                desMirPos = numpy.asarray([x, y, z], dtype=float)
                # put back into mirror coordinate system (vertex as origin)
                desMirPos = desMirPos + baseGimb
                phys = act.physFromMirPos(desMirPos)
                physList.append(phys)
                
            else:
                desMirPos = numpy.dot(rotMat, act.mirPos)
                desMirPos = desMirPos + offsets
                phys = (act.physFromMirPos(desMirPos))
                physList.append(phys)        
        return numpy.asarray(physList, dtype=float)

         


