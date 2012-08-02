#!/usr/bin/env python
import unittest
import numpy
import math
import time

from data import genMirData
import mirrorCtrl
import RO.Astro.Tm

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

#Define maximum allowable fitOrient error, reusing from mirror.py, not using z rot
# MaxOrientErr = numpy.array([0.0001, 5e-8, 5e-8, 0.0001, 0.0001])
MaxOrientErr = numpy.array([.1 * MMPerMicron, .01 * RadPerArcSec, .01 * RadPerArcSec, .1 * MMPerMicron, .1 * MMPerMicron, 1*RadPerDeg])
MaxMountErr = 0.001

# construct range of orients to test +/- (0-25mm, 0-2 degrees)
maxDist = 25.0 # mm
maxTilt = 2.0 * RadPerDeg # radians
resolution = 5 # was 21
# dist_range = numpy.linspace(-maxDist, maxDist, resolution)
# ang_range = numpy.linspace(-maxTilt, maxTilt, resolution)
# ranges = [dist_range, ang_range, ang_range, dist_range, dist_range]
MaxOrient = (maxDist, maxTilt, maxTilt, maxDist, maxDist)
orientRange = [MaxOrient]
for ind in range(len(MaxOrient)):
    for mult in (-1, -0.1, 0.1, 1):
        fitOrient = [0]*5
        fitOrient[ind] = MaxOrient[ind] * mult
        orientRange.append(fitOrient)

# orientRange = numpy.zeros((resolution * 5, 5))
# for ind, rng in enumerate(ranges):
#     orientRange[ind*resolution:ind*resolution+resolution, ind] = rng
# 
# # for 35m tert, don't command translations
# tertOrientRange = orientRange[0:resolution*1,:]
# # construct a set of random orientations.
# num=5
# orientRand = numpy.zeros((num, 5))
# distRand = numpy.random.random_sample(num) * maxDist # pist, transXY
# tiltRand = numpy.random.random_sample(num) * maxTilt
# orientRand[:,[0, 3, 4]] = numpy.vstack((distRand, distRand, distRand)).T
# orientRand[:,[1, 2]] = numpy.vstack((tiltRand, tiltRand)).T

# choose what type of orientation set you want to use in tests, right now there are:
# orientRange and orientRand
orientList = orientRange
print "Testing %s orientations:" % (len(orientList),)
for orient in orientList:
    print "* ", orient

# construct a list of all mirrors, with all actuator options.
# from genMirData:
# prim25List[8]: includes 2 faked fixed link versions and faked encoders
# sec25List[4]: includes a faked fixed link and faked encoders
# sec35List[4]: includes a faked fixed link and faked encoders
# tert35List[2]: 


prim25 = [mirrorCtrl.DirectMirror(*input) for input in genMirData.prim25List]
secCtrMirZ = -135.70
secCtrBaseZ = -178.40
sec25 = [mirrorCtrl.TipTransMirror(secCtrMirZ, secCtrBaseZ, *input) for input in genMirData.sec25List]
sec35 = [mirrorCtrl.DirectMirror(*input) for input in genMirData.sec35List]
tert35 = [mirrorCtrl.DirectMirror(*input) for input in genMirData.tert35List]
# choose which mirrors you want to include in the tests
mirList = prim25 + sec25 + sec35 + tert35
print 'mirList len: ', len(mirList)
############################# TESTS #####################################

class MirTests(unittest.TestCase):
    """Tests for mirrors
    """
    def testRoundTripAct(self):
        linkType = 'Act'
        errLog = self._roundTrip(linkType)
        if len(errLog) > 0:
            self._errLogPrint(errLog, linkType)
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')

    def testRoundTripEnc(self):
        linkType = 'Enc'
        errLog = self._roundTrip(linkType)
        if len(errLog) > 0:
            self._errLogPrint(errLog, linkType)
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')
        
    def _roundTrip(self, linkType):
    
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

                    # to make the initial orientation guess for the fitter,
                    # add noise to the user-adjustable components of adjOrient
                    initOrient = numpy.array(adjOrient, copy=True)
                    initOrient[0:numAdjOrient] += ((2 * numpy.random.random_sample(numAdjOrient)) - 1) * OrientNoiseAmplitude[0:numAdjOrient]
#                     print "adjOrient=", adjOrient
#                     print "initOrient=", initOrient
                    
                    fitOrient = orientFromMount(desMount, initOrient) 
                    #fitOrient = orientFromMount(desMount, adjOrient) # no noise
                    fitMount = mountFromOrient(fitOrient)
                except RuntimeError as er:
                    errLog.append(self._fmtRunTimeErr(er, desOrient, mirInd))
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
        fileDate = time.localtime()
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
        orientErr = numpy.abs(numpy.subtract(adjOrient, fitOrient))
        mountErr = numpy.abs(numpy.subtract(desMount, fitMount))
        if numpy.any(orientErr > MaxOrientErr) or numpy.any(mountErr > MaxMountErr):
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
