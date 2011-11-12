#!/usr/bin/env python
import unittest
import numpy
import math

import parseMirFile_adjBase
import parseMirFile_adjLen
import mirror

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

# construct range of orients to test (0-25mm, 0-2 degrees)
resolution = 10
dist_range = numpy.linspace(0, 25, resolution)
ang_range = numpy.linspace(0, 2 * RadPerDeg, resolution)
ranges = [dist_range, ang_range, ang_range, dist_range, dist_range]
orientRange = numpy.zeros((resolution * 5, 5))
for ind, rng in enumerate(ranges):
    orientRange[ind*resolution:ind*resolution+resolution, ind] = rng

class mirTest(unittest.TestCase):
    def _printErr(self, orientTest, mount, orient1, mount1):
        # commanded orient is only 5 axes, add a zero for rotation for math to work
        orientTest = numpy.hstack((numpy.array(orientTest),0.))
        orientErr = numpy.abs(orientTest - orient1)
        mountErr = numpy.abs(numpy.asarray(mount, dtype=float) - numpy.asarray(mount1, dtype=float))
        if len(mountErr) < 6:
            # for printing to work
            zstack = numpy.zeros(6-len(mountErr))
            mountErr = numpy.hstack((mountErr, zstack))
        str = '%9.2f %9.3f %9.3f %9.2f %9.2f %9.3f\
               %9.2f %9.3f %9.3f %9.2f %9.2f %9.3f\
               %9.2f %9.3f %9.3f %9.2f %9.2f %9.3f' %\
                (orientTest[0], orientTest[1]/RadPerArcSec, orientTest[2]/RadPerArcSec, 
                orientTest[3], orientTest[4], orientTest[5]/RadPerArcSec, 
                orientErr[0], orientErr[1]/RadPerArcSec, orientErr[2]/RadPerArcSec, 
                orientErr[3], orientErr[4], orientErr[5]/RadPerArcSec, 
                mountErr[0], mountErr[1], mountErr[2], 
                mountErr[3], mountErr[4], mountErr[5])
        print str
        
    def test(self):
        print str
        for orientTest in orientRange:
            mount = self.mir.actuatorMountFromOrient(orientTest)
            # from encoders this time
            orient1 = self.mir.orientFromEncoderMount(mount)
            mount1 = self.mir.actuatorMountFromOrient(orient1)
            self._printErr(orientTest, mount, orient1, mount1)
                    
class prim25_adjLen(mirTest):
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actList
        self.mir = mirror.DirectMirror(actList, [])

class prim25_enc_adjLen(mirTest):
    # contrived encoder locations
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actList
        encList = parseMirFile_adjLen.mir25_prim_encList
        self.mir = mirror.DirectMirror(actList, [], encList)

class prim25_fix_adjLen(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actListFixed
        encList = actList[0:5]
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

class prim25_adjBase(mirTest):
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actList
        self.mir = mirror.DirectMirror(actList, [])

class prim25_enc_adjBase(mirTest):
    # contrived encoder locations
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actList
        encList = parseMirFile_adjBase.mir25_prim_encList
        self.mir = mirror.DirectMirror(actList, [], encList)

class prim25_fix_adjBase(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actListFixed
        encList = actList[0:5]
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

class sec25_adjBase(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        SecCtrMirZ = -135.70
        SecCtrBaseZ = -178.40
        # new style encoders
        actList = parseMirFile_adjBase.mir25_sec_actList
        self.mir = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])
        
class sec25_adjLen(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        SecCtrMirZ = -135.70
        SecCtrBaseZ = -178.40
        # new style encoders
        actList = parseMirFile_adjLen.mir25_sec_actList
        self.mir = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])

class sec35_adjLen(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjLen.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])
        
class sec35_adjBase(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])            

class tert35_adjLen(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

class tert35_adjBase(mirTest):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

if __name__ == '__main__':
     suite = unittest.TestLoader().loadTestsFromTestCase(tert35_adjBase)
     unittest.TextTestRunner(verbosity=2).run(suite)
