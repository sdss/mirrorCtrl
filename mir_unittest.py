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

############################# TESTS #####################################

class roundTrip(unittest.TestCase):
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
        str = 'Printing:  Orient Input(6)   Orient Diff(6)   Mount Diff(6)'
        print
        print str
        for orientTest in orientRange:
            mount = self.mir.actuatorMountFromOrient(orientTest)
            orient1 = self.mir.orientFromEncoderMount(mount)
            mount1 = self.mir.actuatorMountFromOrient(orient1)
            self._printErr(orientTest, mount, orient1, mount1)
            
class mountDiff(unittest.TestCase):
    def _printErr(self, orientTest, mount1, mount2):
        # commanded orient is only 5 axes, add a zero for rotation for math to work
        orientTest = numpy.hstack((numpy.array(orientTest),0.))
        mountErr = numpy.abs(numpy.asarray(mount1, dtype=float) - numpy.asarray(mount2, dtype=float))
        if len(mountErr) < 6:
            # for printing to work
            zstack = numpy.zeros(6-len(mountErr))
            mountErr = numpy.hstack((mountErr, zstack))
            
        str = '%9.2f %9.3f %9.3f %9.2f %9.2f %9.3f\
               %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f' %\
                (orientTest[0], orientTest[1]/RadPerArcSec, orientTest[2]/RadPerArcSec, 
                orientTest[3], orientTest[4], orientTest[5]/RadPerArcSec,  
                mountErr[0], mountErr[1], mountErr[2], 
                mountErr[3], mountErr[4], mountErr[5])
                
        print str
       
       
    def test(self):
        str = 'Printing:  Orient Input(6)   Mount Diff(6)'
        print
        print str

        for orientTest in orientRange:
            mount1 = self.mir1.actuatorMountFromOrient(orientTest)
            mount2 = self.mir2.actuatorMountFromOrient(orientTest)
            self._printErr(orientTest, mount1, mount2)        
                    
                    
################################################################################


######################### roundTrip setUps #####################################

class prim25_adjLen(roundTrip):
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actList
        self.mir = mirror.DirectMirror(actList, [])

class prim25_enc_adjLen(roundTrip):
    # contrived encoder locations
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actList
        encList = parseMirFile_adjLen.mir25_prim_encList
        self.mir = mirror.DirectMirror(actList, [], encList)

class prim25_fix_adjLen(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actListFixed
        encList = actList[0:5]
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

class prim25_adjBase(roundTrip):
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actList
        self.mir = mirror.DirectMirror(actList, [])

class prim25_enc_adjBase(roundTrip):
    # contrived encoder locations
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actList
        encList = parseMirFile_adjBase.mir25_prim_encList
        self.mir = mirror.DirectMirror(actList, [], encList)

class prim25_fix_adjBase(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actListFixed
        encList = actList[0:5]
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])

class sec25_adjBase(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        SecCtrMirZ = -135.70
        SecCtrBaseZ = -178.40
        # new style encoders
        actList = parseMirFile_adjBase.mir25_sec_actList
        self.mir = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])
        
class sec25_adjLen(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        SecCtrMirZ = -135.70
        SecCtrBaseZ = -178.40
        # old style encoders
        actList = parseMirFile_adjLen.mir25_sec_actList
        self.mir = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])

class sec35_adjLen(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])
        
class sec35_adjBase(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir35_sec_actList
        self.mir = mirror.DirectMirror(actList[0:5], [actList[5]])            

class tert35_adjLen(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjBase.mir35_tert_actList
        self.mir = mirror.DirectMirror(actList[0:3], actList[3:])

class tert35_adjBase(roundTrip):
    # holding one z rot link as a fixed link
    def setUp(self):
        # new style encoders
        actList = parseMirFile_adjBase.mir35_tert_actList
        self.mir = mirror.DirectMirror(actList[0:3], actList[3:])
        
######################### mountDiff setUps #####################################
        
        
class prim25_mnt(mountDiff):
    def setUp(self):
        # old style encoders
        actList = parseMirFile_adjLen.mir25_prim_actList
        self.mir1 = mirror.DirectMirror(actList, [])
        # new style encoders
        actList = parseMirFile_adjBase.mir25_prim_actList
        self.mir2 = mirror.DirectMirror(actList, [])
        
class sec25_mnt(mountDiff):
    def setUp(self):
        SecCtrMirZ = -135.70
        SecCtrBaseZ = -178.40

        # old style encoders
        actList = parseMirFile_adjLen.mir25_sec_actList
        self.mir1 = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])
        actList = parseMirFile_adjBase.mir25_sec_actList
        self.mir2 = mirror.TipTransMirror(SecCtrMirZ, SecCtrBaseZ, actList[0:5], [actList[5]])
        
if __name__ == '__main__':
    # choose which mirror you want to test         
    suite = unittest.TestLoader().loadTestsFromTestCase(sec25_adjLen)
    unittest.TextTestRunner(verbosity=2).run(suite)
