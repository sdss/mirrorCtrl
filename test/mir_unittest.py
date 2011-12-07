#!/usr/bin/env python
import unittest
import numpy
import math
import time

from data import genMirData
import mirror

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

#Define maximum allowable orient error, reusing from mirror.py, not using z rot
# maxOrientErr = numpy.array([0.0001, 5e-8, 5e-8, 0.0001, 0.0001])
maxOrientErr = numpy.array([.1 * MMPerMicron, .01 * RadPerArcSec, .01 * RadPerArcSec, .1 * MMPerMicron, .1 * MMPerMicron, 1*RadPerDeg])
maxMountErr = 0.001

# construct range of orients to test +/- (0-25mm, 0-2 degrees)
maxDist = 25. #mm
maxTilt = 2. * RadPerDeg # 2 degrees
resolution = 21
dist_range = numpy.linspace(-maxDist, maxDist, resolution)
ang_range = numpy.linspace(-maxTilt, maxTilt, resolution)
ranges = [dist_range, ang_range, ang_range, dist_range, dist_range]
orientRange = numpy.zeros((resolution * 5, 5))
for ind, rng in enumerate(ranges):
    orientRange[ind*resolution:ind*resolution+resolution, ind] = rng

# for 35m tert, don't command translations
tertOrientRange = orientRange[0:resolution*1,:]
# construct a set of random orientations.
num=5
orientRand = numpy.zeros((num, 5))
distRand = numpy.random.random_sample(num) * maxDist # pist, transXY
tiltRand = numpy.random.random_sample(num) * maxTilt
orientRand[:,[0, 3, 4]] = numpy.vstack((distRand, distRand, distRand)).T
orientRand[:,[1, 2]] = numpy.vstack((tiltRand, tiltRand)).T

# choose what type of orientation set you want to use in tests, right now there are:
# orientRange and orientRand
orientList = orientRange

# construct a list of all mirrors, with all actuator options.
# from genMirData:
# prim25List[8]: includes 2 faked fixed link versions and faked encoders
# sec25List[4]: includes a faked fixed link and faked encoders
# sec35List[4]: includes a faked fixed link and faked encoders
# tert35List[2]: 


prim25 = [mirror.DirectMirror(*input) for input in genMirData.prim25List]
secCtrMirZ = -135.70
secCtrBaseZ = -178.40
sec25 = [mirror.TipTransMirror(secCtrMirZ, secCtrBaseZ, *input) for input in genMirData.sec25List]
sec35 = [mirror.DirectMirror(*input) for input in genMirData.sec35List]
tert35 = [mirror.DirectMirror(*input) for input in genMirData.tert35List]
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
                
        errLog=[]
        for mirIter, mir in enumerate(mirList):
            if linkType == 'Enc':
                mountFromOrient = mir.encoderMountFromOrient
                orientFromMount = mir.orientFromEncoderMount
            elif linkType == 'Act':
                mountFromOrient = mir.actuatorMountFromOrient
                orientFromMount = mir.orientFromActuatorMount
                
            for orientIn in orientList:
                try:
                    mnt, adjOrient = mountFromOrient(orientIn, return_adjOrient=True)
                    # remove adjOrient from arguments to default to an initial guess of zeros
                    # for the minimization
                    # add some noise then use noisyOrient as initial guess for round trip
                    adjOrient = numpy.asarray(adjOrient, dtype=float)
                    noiseAmp = numpy.asarray([maxDist, maxTilt, maxTilt, 
                                                maxDist, maxDist, 0.], dtype=float) / 10.
                    # noise is: (flat distribution between -1 and 1) * noiseAmp + adjOrient                            
                    noisyOrient = (2 * numpy.random.random_sample(6,) - 1) * noiseAmp + adjOrient 
                    
                    orient = orientFromMount(mnt, noisyOrient) 
                    #orient = orientFromMount(mnt, adjOrient) # no noise
                    mnt2 = mountFromOrient(orient)
                except RuntimeError as er:
                    errLog.append(self._fmtRunTimeErr(er, orientIn, mirIter))
                    continue
                orientErr, mountErr, failStr = self._checkOutput(adjOrient, orient, mnt, mnt2, mirIter)
                if (True in (orientErr > maxOrientErr)) or (True in (mountErr > maxMountErr)):
                    errLog.append(failStr)
        return errLog            

    def _fmtRunTimeErr(self, error, orientIn, mirIter):
        """For catching and printing a RuntimeError
        """
        errstr =' Mirror number: %2.0f orientIn: %10.4f %10.4f %10.4f %10.4f %10.4f' %\
                               (mirIter, orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
                                orientIn[3], orientIn[4]) + str(error) + '\n'
        return errstr
        
    def _errLogPrint(self, errLog, linkType):
        """Print the error log to a file
        """
        fileDate = time.localtime()
        # minutes and seconds appended to filename
        fileName = 'Errors' + linkType + '_' + str(fileDate[4]) + str(fileDate[5]) +'.log'
        with open(fileName, 'w') as f:
            for line in errLog:
                f.write(line)

    def _checkOutput(self, orientIn, orientOut, mountIn, mountOut, mirIter):
        """This checks if the value abs(orientIn - orientOut) is within
        the limit defined by maxOrientErr.  If it isn't the test fails,
        and the orientIn and Errors are printed """
        # orientIn = numpy.asarray(orientIn[0:5], dtype=float) # only 5 axes (no rotZ)
        # orientOut = numpy.asarray(orientOut[0:5], dtype=float) # only 5 axes (no rotZ)
        orientErr = numpy.abs(orientIn - orientOut) # 6 axes
        mountIn = numpy.asarray(mountIn, dtype=float)
        mountOut = numpy.asarray(mountOut, dtype=float)
        mountErr = numpy.abs(mountIn - mountOut)
        # append zero error to make len(mountErr) == 6
        if len(mountErr) < 6:
            mountErr = numpy.hstack((mountErr, numpy.zeros(6-len(mountErr)))) # for printing zero pad
        failStr = 'Mirror number: %s orientAdj(5):  %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\
        orientErr(5): %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f  mountErr(6): %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f\n' %\
               (mirIter, orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
               orientIn[3], orientIn[4], orientIn[5]/RadPerArcSec, orientErr[0],
               orientErr[1]/RadPerArcSec, orientErr[2]/RadPerArcSec,
               orientErr[3], orientErr[4], orientErr[5]/RadPerArcSec, mountErr[0],
               mountErr[1], mountErr[2],
               mountErr[3], mountErr[4], mountErr[5])
        return orientErr, mountErr, failStr
                            
                
if __name__ == '__main__':
    unittest.main()
