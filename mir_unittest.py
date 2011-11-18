#!/usr/bin/env python
import unittest
import numpy
import math

import genMirData
import mirror

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

#Define maximum allowable orient error, reusing from mirror.py, not using z rot
# maxOrientErr = numpy.array([0.0001, 5e-8, 5e-8, 0.0001, 0.0001])
maxOrientErr = numpy.array([0.1, 5e-3, 5e-3, 0.1, 0.1])
maxMountErr = 0.1

# construct range of orients to test (0-25mm, 0-2 degrees)
maxDist = 25 #mm
maxTilt = 2 # degrees
resolution = 5
dist_range = numpy.linspace(0, maxDist, resolution)
ang_range = numpy.linspace(0, maxTilt * RadPerDeg, resolution)
ranges = [dist_range, ang_range, ang_range, dist_range, dist_range]
orientRange = numpy.zeros((resolution * 5, 5))
for ind, rng in enumerate(ranges):
    orientRange[ind*resolution:ind*resolution+resolution, ind] = rng
    
# construct a set of random orientations.
num=5
orientRand = numpy.zeros((num, 5))
distRand = numpy.random.random_sample(num) * maxDist # pist, transXY
tiltRand = numpy.random.random_sample(num) * maxTilt * RadPerDeg
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

############################# TESTS #####################################

class MirTests(unittest.TestCase):
    """Tests for mirrors
    """

    def _roundTrip(self, linkType):
        for mir in mirList:
            for orientIn in orientList:
                try:
                    mnt = mir.actuatorMountFromOrient(orientIn)
                    orient = mir.orientFromActuatorMount(mnt)
                    mnt2 = mir.actuatorMountFromOrient(orient)
                except RuntimeError as er:
                    print str(er) + '  orientIn: %10.4f %10.4f %10.4f %10.4f %10.4f  mirIter: %s' %\
                               (orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
                                orientIn[3], orientIn[4], mirIter)
                    continue
                orientErr, failStr = self._checkOrient(orientIn, orient)
                if True in (orientErr > maxOrientErr):
                    print failStr
                    break 
                mountErr, failStr = self._checkMount(orientIn, mnt, mnt2)
                if True in (mountErr > maxMountErr):
                    print failStr
                    break 

    def testRoundTripAct(self):
        for orientIn in orientList:
            try:
                mnt = mir.actuatorMountFromOrient(orientIn)
                orient = mir.orientFromActuatorMount(mnt)
                mnt2 = mir.actuatorMountFromOrient(orient)
            except RuntimeError as er:
                print str(er) + '  orientIn: %10.4f %10.4f %10.4f %10.4f %10.4f  mirIter: %s' %\
                           (orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
                            orientIn[3], orientIn[4], mirIter)
                continue
            orientErr, failStr = self._checkOrient(orientIn, orient)
            if True in (orientErr > maxOrientErr):
                print failStr
                break 
            mountErr, failStr = self._checkMount(orientIn, mnt, mnt2)
            if True in (mountErr > maxMountErr):
                print failStr
                break          

    def testRoundTripEnc(self):
        for orientIn in orientList:
            try:
                mnt = mir.encoderMountFromOrient(orientIn)
                orient = mir.orientFromEncoderMount(mnt)
                mnt2 = mir.encoderMountFromOrient(mnt)
            except RuntimeError as er:
                print str(er) + '  orientIn: %10.4f %10.4f %10.4f %10.4f %10.4f  mirIter: %s' %\
                           (orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
                            orientIn[3], orientIn[4], mirIter)
                continue
            orientErr, failStr = self._checkOrient(orientIn, orient)
            if True in (orientErr > maxOrientErr):
                print failStr
                break 
            mountErr, failStr = self._checkMount(orientIn, mnt, mnt2)
            if True in (mountErr > maxMountErr):
                print failStr
                break     
                
    def _checkOrient(self, orientIn, orientOut):
        """This checks if the value abs(orientIn - orientOut) is within
        the limit defined by maxOrientErr.  If it isn't the test fails,
        and the orientIn and Errors are printed """
        orientIn = numpy.asarray(orientIn[0:5], dtype=float) # only 5 axes (no rotZ)
        orientOut = numpy.asarray(orientOut[0:5], dtype=float) # only 5 axes (no rotZ)
        orientErr = numpy.abs(orientIn - orientOut)
        failStr = 'orientIn(5):  %9.4f %9.4f %9.4f %9.4f %9.4f \n orientErr(5): %9.4f %9.4f %9.4f %9.4f %9.4f' %\
               (orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
               orientIn[3], orientIn[4], orientErr[0],
               orientErr[1]/RadPerArcSec, orientErr[2]/RadPerArcSec,
               orientErr[3], orientErr[4])
        return orientErr, failStr
        #self.assertFalse(True in (orientErr > maxOrientErr), failStr)

    
    def _checkMount(self, orientIn, mountIn, mountOut):
        """This compares the differnce between the mount computed from
        initial orient and mount computed from recovered orient.  This
        will check that the fitting process produced the desired
        orientation, and not an alternate one """
        orientIn = numpy.asarray(orientIn[0:5], dtype=float) # only 5 axes (no rotZ)
        mountIn = numpy.asarray(mountIn, dtype=float)
        mountOut = numpy.asarray(mountOut, dtype=float)
        mountErr = numpy.abs(mountIn - mountOut)
        # append zero error to make len(mountErr) == 6
        if len(mountErr) < 6:
            mountErr = numpy.hstack((mountErr, numpy.zeros(6-len(mountErr))))
        failStr = 'orientIn(5):  %9.4f %9.4f %9.4f %9.4f %9.4f \n mountErr(6): %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f' %\
               (orientIn[0], orientIn[1]/RadPerArcSec, orientIn[2]/RadPerArcSec,
               orientIn[3], orientIn[4], mountErr[0],
               mountErr[1], mountErr[2],
               mountErr[3], mountErr[4], mountErr[5])
        return mountErr, failStr
        #self.assertFalse(True in (mountErr > maxMountErr), failStr)
   

                
                
if __name__ == '__main__':
    for ind, mir in enumerate(mirList):
        print 'Testing Mirror # ', ind    
        unittest.main()
       
                    
################################################################################


#         
# if __name__ == '__main__':
#     # choose which mirror you want to test         
#     suite = unittest.TestLoader().loadTestsFromTestCase(sec25_adjLen)
#     unittest.TextTestRunner(verbosity=2).run(suite)


# old tests    
#     def _printErr(self, orientTest, mount, orient1, mount1):
#         # commanded orient is only 5 axes, add a zero for rotation for math to work
#         orientTest = numpy.hstack((numpy.array(orientTest),0.))
#         orientErr = numpy.abs(orientTest - orient1)
#         mountErr = numpy.abs(numpy.asarray(mount, dtype=float) - numpy.asarray(mount1, dtype=float))
#         if len(mountErr) < 6:
#             # for printing to work
#             zstack = numpy.zeros(6-len(mountErr))
#             mountErr = numpy.hstack((mountErr, zstack))
#         str = '%9.2f %9.3f %9.3f %9.2f %9.2f %9.3f\
#                %9.2f %9.3f %9.3f %9.2f %9.2f %9.3f\
#                %9.2f %9.3f %9.3f %9.2f %9.2f %9.3f' %\
#                 (orientTest[0], orientTest[1]/RadPerArcSec, orientTest[2]/RadPerArcSec, 
#                 orientTest[3], orientTest[4], orientTest[5]/RadPerArcSec, 
#                 orientErr[0], orientErr[1]/RadPerArcSec, orientErr[2]/RadPerArcSec, 
#                 orientErr[3], orientErr[4], orientErr[5]/RadPerArcSec, 
#                 mountErr[0], mountErr[1], mountErr[2], 
#                 mountErr[3], mountErr[4], mountErr[5])
#         print str

