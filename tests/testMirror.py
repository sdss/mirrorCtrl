#!/usr/bin/env python2
from __future__ import division, absolute_import

import unittest
import numpy
import numpy.random
import itertools
import copy
import pickle

# This flag determines the initial guess to solve for orietation
# If False: start from a noisy (but close) guess to the solution
# If True: start from all zeros.
zeroFlag = False

import RO.Astro.Tm
#from data import genMirrors
#from data import loadMirDat
import mirrorCtrl
from mirrorCtrl.const import convOrient2MMRad, MMPerMicron, RadPerArcSec, RadPerDeg
from mirrorCtrl.mirrors import mir25mPrim, mir25mSec, mir35mSec, mir35mTert

import os

#Define maximum allowable fitOrient error, reusing from mirror.py, not using z rot
MaxOrientErr = numpy.array([.1 * MMPerMicron, .01 * RadPerArcSec, .01 * RadPerArcSec, .1 * MMPerMicron, .1 * MMPerMicron, 1*RadPerDeg])
MaxMountErr = 0.05
MaxMountAdjNoAdjErr = 0.5 # um
# construct range of orients to test +/- (0-25mm, 0-2 degrees)
#maxDist = 25.0 # mm
#maxTilt = 2.0 * RadPerDeg # radians
maxDist = 50. * MMPerMicron # mm
maxTilt = 3. * RadPerArcSec # radians
# set number of orientation 'samples' to test (per axis)
resolution = 5 # was 21

MaxOrient = (maxDist, maxTilt, maxTilt, maxDist, maxDist)
orientRange = [MaxOrient]
for ind in range(len(MaxOrient)):
    for mult in (-1, -0.1, 0.1, 1):
        fitOrient = [0]*5
        fitOrient[ind] = MaxOrient[ind] * mult
        orientRange.append(fitOrient)


# choose what type of orientation set you want to use in tests, right now there are:
# orientRange and orientRand
orientList = orientRange
print "Testing %s orientations:" % (len(orientList),)
for orient in orientList:
    print "* ", orient

# mirrors can be viewed using the plotMirror() method, eg: mirrorCtrl.mirrors.mir25mPrim.plotMirror()
# use the non-fictitous mirrors:
mirList = [
    mir25mPrim,
    mir25mSec,
    mir35mSec,
    mir35mTert,
]
print 'mirList len: ', len(mirList)

pwd = os.path.dirname(__file__)
secMoveList = pickle.load(open(os.path.join(pwd, "data/secMoveList.p")))
tertMoveList = pickle.load(open(os.path.join(pwd, "data/tertMoveList.p")))
############################# TESTS #####################################

class MirTests(unittest.TestCase):
    """Tests for mirrors
    """
    def _testBallPark(self, mirror, moveList, fract, adj=True):
        """Using a list of mirror log data from the 3.5m, check
        that the M2 and M3 conversions produced by our models are sane.
        """
        # just take a fraction of moveList
        numpy.random.shuffle(moveList)
        nAxes = len(mirror.actuatorList)
        for move in moveList[:5]: # only test 5 orientations
            desOrient = numpy.asarray(move["desOrient"])
            desOrient = desOrient
            encMount, adjOrient = mirror.encoderMountFromOrient(convOrient2MMRad(desOrient), return_adjOrient=True, adjustOrient = adj)
            actMountLog = numpy.asarray([mount/(2*act.maxMount) for mount, act in itertools.izip(move["actMount"][:nAxes], mirror.actuatorList[:nAxes])])
            encMountNew = numpy.asarray([mount/(2*act.maxMount) for mount, act in itertools.izip(encMount, mirror.actuatorList[:nAxes])])
            encMountNew = encMountNew[:nAxes]
            mountDiff = numpy.abs(actMountLog-encMountNew)
            self.assertTrue(numpy.all(mountDiff < fract)) # lengths accurate to < 50 microns

    def testSecBallPark(self):
        self._testBallPark(mirrorCtrl.mirrors.mir35mSec, secMoveList, .02)

    def testTertBallPark(self):
        mirror = mirrorCtrl.mirrors.mir35mTert
        self._testBallPark(mirror, tertMoveList, .14)

    def testTertNoAdjBallPark(self):
        #mirror = mirDict35Adj["tert"]
        mirror = mirrorCtrl.mirrors.mir35mTert
        self._testBallPark(mirror, tertMoveList, .025, False)

    def testRoundTripAct(self):
        # test all orientations in orientList
        linkType = 'Act'
        errLog = self._roundTrip(linkType, orientList)
        if len(errLog) > 0:
            self._errLogPrint(errLog, linkType)
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')

    def testRoundTripEnc(self):
        # just check one orientation, the first in orientList
        linkType = 'Enc'
        errLog = self._roundTrip(linkType, [orientList[0]])
        if len(errLog) > 0:
            self._errLogPrint(errLog, linkType)
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')

    def testMountAdj(self):
        # check that adjusted orientations do not cause a large discrepancy in mount lengths
        errLog=[]
        for ind, mir in enumerate(mirList[:3]): # exclude the 3.5 M3 (due to large induced translations)
            for orient in orientList:
                for getMount, links in itertools.izip([mir.actuatorMountFromOrient, mir.encoderMountFromOrient], [mir.actuatorList, mir.encoderList]):
                    scaleBy = numpy.asarray([link.scale for link in links]) # mount units/um
                    mountAdj = numpy.asarray(getMount(orient))/scaleBy
                    mountNoAdj = numpy.asarray(getMount(orient, adjustOrient = False))/scaleBy
                    mountErr = numpy.abs(mountAdj-mountNoAdj) # in um (scaled)
                    if True in (mountErr > MaxMountAdjNoAdjErr):
                        errStr = 'mir %i mount discrepancy! %s (um err), %s (orientation)' % (ind, str(mountErr), str(orient))
                        errLog.append(errStr)
        if len(errLog) > 0:
            self._errLogPrint(errLog, 'adj_vs_noadj')
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')

    def testOverConstraint(self, mirList=mirList):
        """Simultaneously place a fixed link exactly in place of an adjustable link
        and command a move.  Choose the first actuator in each mirror.
        """
        orient = [maxDist*0.5, maxTilt*0.2, -maxTilt*0.6, maxDist*0.5, -maxDist*0.7] # chosen randomly by me
        mirs = []
        for mir in mirList:
            if len(mir.fixedLinkList) == 0:
                continue # no fixed links for this mirror
            # change the position of the first fixed link to match the position of the
            # first actuator
            mirCp = copy.deepcopy(mir)
            mirCp.fixedLinkList[0].mirPos = mirCp.actuatorList[0].mirPos
            mirCp.fixedLinkList[0].basePos = mirCp.actuatorList[0].basePos
            for link in mirCp.actuatorList:
                link.maxMount = numpy.inf
                link.minMount = numpy.inf * -1.
            mirs.append(mirCp)
        self.expectMountDiscrepancy(mirs, orient)

    def expectMountDiscrepancy(self, mirList, orient):
        for mir in mirList:
            scaleBy = numpy.asarray([link.scale for link in mir.actuatorList]) # mount units/um
            mtAdj = numpy.asarray(mir.actuatorMountFromOrient(orient))/scaleBy
            mtNoAdj = numpy.asarray(mir.actuatorMountFromOrient(orient, adjustOrient = False))/scaleBy
            mountErr = numpy.abs(mtAdj-mtNoAdj) # in um (scaled)
            self.assertTrue(True in (mountErr > MaxMountAdjNoAdjErr))


    def _roundTrip(self, linkType, orientList):

        OrientNoiseAmplitude = numpy.array(MaxOrient) * 0.1
        errLog=[]
        for mirInd, mir in enumerate(mirList):
            print "Testing mirror %s; numAdjOrient=%s" % (mirInd, mir.numAdjOrient)
            if linkType == 'Enc':
                mountFromOrient = mir.encoderMountFromOrient
                orientFromMount = mir.orientFromEncoderMount
            elif linkType == 'Act':
                mountFromOrient = mir.actuatorMountFromOrient
                orientFromMount = mir.orientFromActuatorMount

            numAdjOrient = mir.numAdjOrient
            testedOrientSet = set() # a set of orientations truncated to only the useful values
            for desOrient in orientList:
                usefulOrient = tuple(desOrient[0:numAdjOrient])
                if usefulOrient in testedOrientSet:
                    continue
                testedOrientSet.add(usefulOrient)
                try:
                    desMount, adjOrient = mountFromOrient(desOrient, return_adjOrient=True)

                    # to make the initial orientation guess for the fitter
                    if zeroFlag == True:
                        # choose an initial guess of all zeros
                        initOrient=numpy.zeros(numAdjOrient)
                    else:
                        # add noise to the user-adjustable components of adjOrient
                        initOrient = numpy.array(adjOrient, copy=True)
                        numpy.random.seed(0) # gen same random numbers each time
                        initOrient[0:numAdjOrient] += ((2 * numpy.random.random_sample(numAdjOrient)) - 1) * OrientNoiseAmplitude[0:numAdjOrient]
#                     print "adjOrient=", adjOrient
#                     print "initOrient=", initOrient

                    fitOrient = orientFromMount(desMount, initOrient)
                    #fitOrient = orientFromMount(desMount, adjOrient) # no noise
                    fitMount = mountFromOrient(fitOrient)
                except RuntimeError as e:
                    errLog.append(self._fmtRunTimeErr(e, desOrient, mirInd))
                    print errLog[-1]
                    continue
                errStr = self._checkFitErr(mir, mirInd, desOrient, adjOrient, initOrient, fitOrient, desMount, fitMount)
                if errStr:
                    errLog.append(errStr)
                    print errStr
        return errLog

    def _fmtRunTimeErr(self, error, desOrient, mirInd):
        """For catching and printing a RuntimeError
        """
        errstr =' Mirror number: %2.0f desOrient: %10.4f %10.4f %10.4f %10.4f %10.4f' %\
                               (mirInd, desOrient[0], desOrient[1]/RadPerArcSec, desOrient[2]/RadPerArcSec,
                                desOrient[3], desOrient[4]) + str(error) + '\n'
        return errstr

    def _errLogPrint(self, errLog, linkType):
        """Print the error log to a file
        """
        #fileDate = time.localtime()
        # minutes and seconds appended to filename
        fileName = 'Errors' + linkType + '_' + RO.Astro.Tm.isoDateTimeFromPySec(nDig=0, useGMT=False) + ".log"
        with open(fileName, 'w') as f:
            f.write("\n".join(errLog))

    def _checkFitErr(self, mir, mirInd, desOrient, adjOrient, initOrient, fitOrient, desMount, fitMount):
        """Check fit error and return a str describing the problem, or "" if OK

        Inputs:
        - mir: mirror
        - mirInd: mirror index
        - desOrient: desired orientation before adjustment
        - adjOrient: desired orientation after adjusting constrained axes
        - initOrient: initial guess for fit function
        - fitOrient: actual fit orientation
        - desMount: desired mount (computed from desOrient)
        - fitMount: fit mount (computed from fitOrient)
        """
        # check that the user command-able orientation values have not changed (others are allowed to vary)
        desVsFit = numpy.abs(numpy.subtract(desOrient[:mir.numAdjOrient], fitOrient[:mir.numAdjOrient]))
        orientErr = numpy.abs(numpy.subtract(adjOrient, fitOrient))
        mountErr = numpy.abs(numpy.subtract(desMount, fitMount))
        if numpy.any(orientErr > MaxOrientErr) or numpy.any(desVsFit > MaxOrientErr[:mir.numAdjOrient]) or numpy.any(mountErr > MaxMountErr):
            fa = FormatArr()

            return "Mirror ind=%s; numAdjOrient=%s; desOrient=%s; adjOrient=%s; initOrient=%s; orientErr=%s; mountErr=%s" % \
                   (mirInd, mir.numAdjOrient, fa(desOrient), fa(adjOrient), fa(initOrient), fa(orientErr), fa(mountErr))
        return ""

class FormatArr(object):
    def __init__(self, width=6, ndig=4):
        self.fmtStr = "%%%d.%df" % (width, ndig)
    def __call__(self, arr):
        strList = [self.fmtStr % (val,) for val in arr]
        return ", ".join(strList)

if __name__ == '__main__':
    unittest.main()
