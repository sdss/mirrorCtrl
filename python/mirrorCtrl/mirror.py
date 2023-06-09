from __future__ import division, absolute_import
"""Mirror

The units for orientation are mm and radians (not user-friendly units, but best for computation).
"""
import collections
import itertools
import math

import numpy
import scipy.optimize
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D

__all__ = ["MirrorBase", "DirectMirror", "TipTransMirror"]

## Tolerance parameter used in scipy minimization routine
_FitTol = 1e-8
## Maximum iterations to user in scipy minimization routine
_MaxIter = 10000
## Orientation of mirror; units are mm and radians
Orientation = collections.namedtuple("Orientation", ["piston", "tiltX", "tiltY", "transX", "transY", "rotZ"])
## A zero orientation
ZeroOrientation = Orientation(0, 0, 0, 0, 0, 0)

RadPerDeg = math.pi / 180.0


class MirrorBase(object):
    """Base class for mirrors
    """
    def __init__(self, actuatorList, fixedLinkList,
        encoderList=None, minCorrList=None, maxCorrList=None, name=None):
        """Construct a MirrorBase

        @param[in] actuatorList  List of actuators that support the mirrorCtrl.
        @param[in] fixedLinkList  List of fixed-length links that support the mirror
        @param[in] encoderList   Encoders associated with actuators. None if there are no encoders,
            else a list of items: each an encoder or None if the associated
            actuator has no encoder.
        @param[in] minCorrList  if encoderList is not None: specifies the maximum encoder error to correct (steps);
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        @param[in] maxCorrList  if encoderList is not None: specifies the maximum encoder error to correct (steps);
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        @param[in] name  Mirror name, if you care to specify. Choose from 'Prim', 'Sec', or 'Tert'
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

        if encoderList is None:
            self.hasEncoders = False
            self.encoderList = actuatorList
        else:
            self.hasEncoders = True
            if len(encoderList) != len(actuatorList):
                raise RuntimeError("encoderList must contain %s encoders; has %s" % \
                    (len(actuatorList), len(encoderList)))
            if len(minCorrList) != len(encoderList):
                raise RuntimeError("len(encoderList) = %d != %d = len(minCorrList)" % (len(encoderList), len(minCorrList)))
            if len(maxCorrList) != len(encoderList):
                raise RuntimeError("len(encoderList) = %d != %d = len(maxCorrList)" % (len(encoderList), len(maxCorrList)))

            # Populate encoderList with encoders, using actuators in place of 'None' encoder.
            self.encoderList = []
            self.minCorrList = []
            self.maxCorrList = []
            for enc, act, minCorr, maxCorr in itertools.izip(encoderList, actuatorList, minCorrList, maxCorrList):
                if enc == None:
                    self.encoderList.append(act)
                    self.minCorrList.append(0)
                    self.maxCorrList.append(0)
                else:
                    self.encoderList.append(enc)
                    self.minCorrList.append(minCorr)
                    self.maxCorrList.append(maxCorr)
            self.minCorrList = numpy.array(self.minCorrList, dtype=float)
            self.maxCorrList = numpy.array(self.maxCorrList, dtype=float)
        self.mirRad = 500 # mm, for plotMirror

    def plotMirror(self):
        """ Plots links and glass when the mirror is in neutral position,
        mostly a sanity check.
        """
        mirRad = 500 # mm
        fig = matplotlib.pyplot.figure()
        # ax = fig.gca(projection='3d')
        ax = Axes3D(fig)
        theta = numpy.linspace(0., 2 * numpy.pi, 100)
        z = numpy.zeros(100)
        fig.hold()

        # plot the mirror black concentric circles
        for r in numpy.linspace(0., mirRad, 50):
            x = r * numpy.sin(theta)
            y = r * numpy.cos(theta)
            if len(self.fixedLinkList)==3: # mm
                mirRad = 300
                # 3.5m M3 mirror is rotated about x by -45 deg
                phiRad = 90.0 * RadPerDeg
                cosP = math.cos(phiRad)
                sinP = math.sin(phiRad)
                stack = numpy.vstack((x,y,z))
                stack = stack.T
                rotMat = numpy.array([ [1., 0., 0.],
                                       [0., cosP, sinP],
                                       [0., -sinP, cosP] ])
                stackRot = numpy.dot(stack, rotMat)
                x = stackRot[:,0]
                y = stackRot[:,1]
                z = stackRot[:,2]
            ax.plot(x, y, z, 'k')

        # plot actuators blue
        for act in self.actuatorList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])
            y = numpy.array([act.mirPos[1], act.basePos[1]])
            z = numpy.array([act.mirPos[2], act.basePos[2]])
            ax.plot(x, y, z, 'bo-')

        # plot encoders cyan
        for act in self.encoderList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])
            y = numpy.array([act.mirPos[1], act.basePos[1]])
            z = numpy.array([act.mirPos[2], act.basePos[2]])
            ax.plot(x, y, z, 'co-')

        # plot fixed links red
        for act in self.fixedLinkList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])
            y = numpy.array([act.mirPos[1], act.basePos[1]])
            z = numpy.array([act.mirPos[2], act.basePos[2]])
            ax.plot(x, y, z, 'ro-')

        xyrange=(-1.5 * mirRad, 1.5 * mirRad)
        # zrange = (-3 * mirRad, 2 * mirRad)
        matplotlib.pyplot.xlim(xyrange)
        matplotlib.pyplot.ylim(xyrange)
        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlim(xyrange)
        matplotlib.pyplot.show()

    @property
    def numAdjOrient(self):
        """Return the number of adjustable axes of orientation

        Warning: even if rotZ is technically adjustable it is never counted
        (the max returned value is always 5)
        """
        return min(5, 6 - len(self._fixedAxes))

    def orientFromEncoderMount(self, mount, initOrient=ZeroOrientation):
        """Compute mirror orientation from encoder mount lengths

        @param[in] mount  encoder mount positions; one per encoder or actuator if the actuator has no associated encoder
        @param[in] initOrient  initial guess as to orientation; may be incomplete
        """
        if len(mount) != len(self.encoderList):
            raise RuntimeError("Need %s values; got %s" % (len(self.encoderList), len(mount)))
        return self._orientFromMount(mount, self.encoderList, initOrient)

    def orientFromActuatorMount(self, mount, initOrient = ZeroOrientation):
        """Compute mirror orientation from actuator mount lengths

        @param[in] mount  encoder mount positions; one per encoder
        @param[in] initOrient  initial guess as to orientation; may be incomplete
        """
        if len(mount) != len(self.actuatorList):
            raise RuntimeError("Need %s values; got %s" % (len(self.actuatorList), len(mount)))
        return self._orientFromMount(mount, self.actuatorList, initOrient)

    def actuatorMountFromOrient(self, userOrient, return_adjOrient = False, adjustOrient = True):
        """Compute actuator mount lengths from orientation

        @param[in] userOrient  an Orientation or collection of 6 items or fewer in the
            same order. Keywords are also accepted, undefined fields will be
            silently replaced as zeros.
        @param[in] return_adjOrient  bool. Whether or not to return the adjustedOrientation.
        @param[in] adjustOrient  bool.  Whether or not to adjust orientation, to discover a "best-fit" orientation
            that accounts for induced motions due to fixed links

        @return 1 or 2 items:
            - mountList: list of mount coords for actuator/encoders
            - adjOrient: adjusted orient based on mirror fixed links
        """
        if len(userOrient) not in set((0, 1, 3, 5, 6)):
            raise RuntimeError('Input orientation must be a list of numbers of \n\
                                length 0, 1, 3, or 5, 6. Actual length %s' % (len(userOrient)))

        # set all non-supplied orientations to 0.
        orient = numpy.zeros(6, dtype=float)
        orient[0:len(userOrient)] = userOrient
        if adjustOrient:
            orient = self._fullOrient(orient)
        else:
            orient = Orientation(*orient)
        mountList = self._mountFromOrient(orient, self.actuatorList)
        if return_adjOrient == True:
            return mountList, orient
        else:
            return mountList

    def encoderMountFromOrient(self, userOrient, return_adjOrient = False, adjustOrient = True):
        """Compute encoder mount lengths from orientation

        @param[in] userOrient  an Orientation or collection of 6 items or fewer in the
            same order. Keywords are also accepted, undefined fields will be
            silently replaced as zeros.
        @param[in] return_adjOrient  bool. Whether or not to return the adjustedOrientation.
        @param[in] adjustOrient  bool.  Whether or not to adjust orientation, to discover a "best-fit" orientation
        that accounts for induced motions due to fixed links

        @return 1 or 2 items:
            - mountList: list of mount coords for actuator/encoders
            - adjOrient: adjusted orient based on mirror fixed links
        """
        if len(userOrient) not in set((0, 1, 3, 5, 6)):
            raise RuntimeError('Input orientation must be a list of numbers of \n\
                                length 0, 1, 3, or 5, 6. Actual length %s' % (len(userOrient)))

        # set all non-supplied orientations to 0.
        orient = numpy.zeros(6, dtype=float)
        orient[0:len(userOrient)] = userOrient
        if adjustOrient:
            orient = self._fullOrient(orient)
        else:
            orient = Orientation(*orient)
        mountList = self._mountFromOrient(orient, self.encoderList)
        if return_adjOrient == True:
            return mountList, orient
        else:
            return mountList

    def _fullOrient(self, orient):
        """Compute fully specified orientation from a partially specified orientation.

        @param[in] orient  list of orientation values: 0, 1, 3, 5 or 6 values. Missing values are treated as 0.

        Output:
        @ return orient: the full 6-axis orientation as an Orientation. Axes that cannot be controlled
            are set to their constrained value (which should be nearly 0 for a typical mirror).
        """

        # compute constrained axes of orientation
        if self._fixedAxes:
            linkList = self.fixedLinkList
            physMult = self._physMult(linkList)
            # phys lengths should be zero, since we're dealing with fixed length links
            givPhys = numpy.zeros(len(self._fixedAxes))
            # only searching for solutions for the fixed axes
            initOrient = numpy.zeros(len(self._fixedAxes))
            minOut = scipy.optimize.fmin_powell(
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

        @param[in] orient  a 6 element orientation
        @param[in] linkList  a list of link objects for which to
            determin lengths for

        @return mountList: a list of mount lengths (in mount units)
            corresponding to the links provided in linkList
        """
        phys = self._physFromOrient(orient, linkList)
        return [link.mountFromPhys(p) for p, link in itertools.izip(phys, linkList)]

    def _physFromOrient(self, orient, linkList):
        """Compute physical link length from orientation.

        Subclasses must override

        @param[in] orient  a 6 element list/array of orientation values
        @param[in] linkList  a list of link objects for which to
            determin lengths for
        """
        raise NotImplementedError()

    def _orientFromMount(self, mount, linkList, initOrient=ZeroOrientation):
        """Compute orientation from link mount length

        @param[in] mount  link mount length
        @param[in] linkList  list of Links
        @param [in] initOrient: initial guess as to orientation (may be incomplete)

        @return orient, a 6 element orientation
        """
        if len(mount) != len(linkList):
            raise RuntimeError("len(mount)=%s != %s=len(linkList)" % (len(mount), len(linkList)))
        phys = [link.physFromMount(mt) for mt, link in itertools.izip(mount, linkList)]
        return self._orientFromPhys(phys, linkList, initOrient)

    def _physMult(self, linkList):
        """Determine multiplier for each actuator to be used in minimization

        @param[in] linkList  list of links

        @return physMult: error multiplier for each axis.

        Computes the effect of a perturbing each axis of orientation by a given amount
        on each link actuator length. This is used to scale the weighting of physical
        length errors when fitting orientation from physical length.
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

        @param[in] phys  physical position of each actuator or encoder; the position for fixed links will be ignored
        @param[in] linkList  list of actuators or encoders (not fixed links!)
        @param[in] initOrient  initial guess as to orientation; may be incomplete

        @return orient: mirror orientation (an Orientation)
        """
        givPhys = phys[:]
        # we want to compute orient using fixed link constraints
        linkListFull = linkList + self.fixedLinkList
        # now add a phys of zero for fixed links.
        givPhys.extend(numpy.zeros(len(self.fixedLinkList)))
        # Compute physical errors
        physMult = self._physMult(linkListFull)

        orient = numpy.zeros(6, dtype=float)
        orient[0:len(initOrient)] = initOrient

        fullInitOrient = self._fullOrient(orient)

        minOut = scipy.optimize.fmin_powell(
            self._minOrientErr,
            fullInitOrient,
            args = (givPhys, physMult, linkListFull),
            maxiter = _MaxIter,
            ftol = _FitTol,
            disp = 0,
            full_output = True
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

        @param[in] minOrient  an orientation to be solved for. If it is less than 6 elements, then fitAxes
                        must to be specified and must be the same length as minOrient.
        @param[in] givPhys   list of physical link lengths, must be same length as linkList
        @param[in] physMult  list of multipliers for computing errors
        @param[in] linkList  list of links
        @param[in] fitAxes   a Python list of axes indices to solve for.
                        Ignored if minOrient has 6 elements, else must be the same size as minOrient.
        @param[in] fullOrient  6 item collection. This is a 'constant' orientation, minOrient is inserted
                        into specific axes of fullOrient defined by fitAxes. This way we can define
                        a minimization for an arbitrary amount of orientation axes while leaving
                        the constant axes untouched by the fitter.

        @return sum(physMult * physErrSq**2)

        @throw IndexError if fitAxes is not a Python list
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

        @param[in] orient   an Orientation

        @return 2 values:
        - rotMat:  3x3 rotation matrix
        - offsets: 3x1 (x,y,z) offset vector

        Compute matrices used to transform cartesian points on the mirror from
        their location at zero orientation to their location at the specified orientation.
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
    """A mirror supported by 6 actuators or fixed links connected directly to the mirrorCtrl.
    """
    def __init__(self, actuatorList, fixedLinkList, encoderList, minCorrList, maxCorrList, name=None):
        """Construct a Direct mirror

        @param[in] actuatorList  List of actuators that support the mirrorCtrl.
        @param[in] fixedLinkList  List of fixed-length links that support the mirror
        @param[in] encoderList   Encoders associated with actuators. None if there are no encoders,
            else a list of items: each an encoder or None if the associated
            actuator has no encoder.
        @param[in] minCorrList  if encoderList is not None: specifies the minimum actuator error to correct;
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        @param[in] maxCorrList  if encoderList is not None: specifies the maximum actuator error to correct;
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        @param[in] name  Mirror name, if you care to specify. Choose from 'Prim', 'Sec', or 'Tert'
            this is read by the actor and used to generate appriate keywords

        Two configurations are supported:
        - control piston, tip and tilt only (no translation or z rotation)
        - control piston, tip, tilt and translation (no z rotation)
        """
        MirrorBase.__init__(self,
            actuatorList = actuatorList,
            fixedLinkList = fixedLinkList,
            encoderList = encoderList,
            minCorrList = minCorrList,
            maxCorrList = maxCorrList,
            name = name,
        )

    def _physFromOrient(self, orient, linkList):
        """Compute desired physical position of actuators, encoders or fixed
        length links given mirror orientation If a fixed length link is included
        in linkList, the resulting phys value is probably unobtainable. We use
        this fact in minimization routines, to solve for correct orientations
        given the fact that in reality phys = 0 for fixed length links.

        @param[in] orient  mirror orientation (an Orientation)
        @param[in] linkList  list of actuators, encoders, or fixed length links

        @return physList: computed physical distance to the mirror (mm), measured from the neutral
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


class TipTransMirror(MirrorBase):
    """Tip-Trans Mirror.

    A mirror that rides on a central linear bearing;
    the bearing is tipped about a ball joint to translate the mirrorCtrl.
    Actuators 0, 1, 2 are axial actuators attached to the mirrorCtrl.
    Actuators 3, 4 are attached to the linear bearing to tip it.
    Actuator 5 is an antirotation link attached to the mirrorCtrl.
    """
    def __init__(self, ctrMirZ, ctrBaseZ, actuatorList, fixedLinkList,
        encoderList, minCorrList, maxCorrList, name=None):
        """Construct a Tip-Trans Mirror

        @param[in] ctrMirZ  z position of center of mirror in neutral position
        @param[in] ctrBaseZ  z position of center of base (ball joint)
        @param[in] fixedLinkList  List of fixed-length links that support the mirror
        @param[in] encoderList   Encoders associated with actuators. None if there are no encoders,
            else a list of items: each an encoder or None if the associated
            actuator has no encoder.
        @param[in] minCorrList  if encoderList is not None: specifies the maximum actuator error to correct;
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        @param[in] maxCorrList  if encoderList is not None: specifies the maximum actuator error to correct;
            must have the same number of elements as encoderList;
            if encoderList[i] is None then minCorrList[i] is ignored.
            If encoderList is None then this argument is entirely ignored.
        """
        MirrorBase.__init__(self,
            actuatorList = actuatorList,
            fixedLinkList = fixedLinkList,
            encoderList = encoderList,
            minCorrList = minCorrList,
            maxCorrList = maxCorrList,
            name = name,
        )
        self.ctrMirZ = ctrMirZ
        self.ctrBaseZ = ctrBaseZ
#         if len(self.fixedLinkList) != 1:
#             raise RuntimeError('TipTrans Mirror needs one z anti-rotation fixed length link defined\
#                                 %s were supplied' % (len(self.fixedLinkList)))

    def _rotEqPolMat(self, eqAng, polAng):
        """
        This method was translated from src/subr/cnv/roteqpol.for.

        @param[in] eqAng   Equatorial rotation angle (radians) of line in x-y plane (from x to y).
                   This plane of rotation includes this line and z.
        @param[in] polAng  Polar rotation angle (radians) from z axis to the line in the x-y plane.

        @return rotMatEqPol: 3x3 rotation matrix

        Defines a matrix that rotates a 3-vector described by equatorial
        and polar angles as follows: The plane of rotation contains the
        z axis and a line in the x-y plane at angle eqAng from the x
        axis towards y. The amount of rotation is angle polAng from the
        z axis towards the line in the x-y plane.
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

        @param[in] orient   mirror orientation with 6 axes 6 item list:
        @param[in] linkList  list of actuators or encoders

        @return physList[0:5]: delta length for link length (mm) measured from the neutral position.
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
        # apply desired tilts, xy translation and z piston
        mirGimbRot = numpy.dot(rotMat, mirGimb)
        desMirGimb = mirGimbRot + offsets
        # take the base gimbal to be origin
        desMirGimb = desMirGimb - baseGimb
        # get polar (from +Z) and equatorial (from +X, 0 - 2pi rad) angles
        eqAng = numpy.arctan2(desMirGimb[1], desMirGimb[0])
        if eqAng < 0:
            # atan2 is defined from -pi:pi, for spherical coords we want 0:2*pi
            eqAng += 2 * math.pi
        # dot product to find angle wrt z axis
        polAng = numpy.arccos(numpy.dot(desMirGimb / numpy.linalg.norm(desMirGimb), numpy.array([0,0,1])))
        eqPolRot = self._rotEqPolMat(eqAng, polAng)

        physList = []
        for ind, act in enumerate(linkList):
            if ind in [3,4]:
                actUnrot = act.mirPos - baseGimb # rotate about baseGimb
                desMirPos = numpy.dot(eqPolRot, actUnrot) + baseGimb # back to coord sys with origin at mir vertex
                phys = (act.physFromMirPos(desMirPos))
                physList.append(phys)
            else:
                desMirPos = numpy.dot(rotMat, act.mirPos)
                desMirPos = desMirPos + offsets
                phys = (act.physFromMirPos(desMirPos))
                physList.append(phys)

        return numpy.asarray(physList, dtype=float)
