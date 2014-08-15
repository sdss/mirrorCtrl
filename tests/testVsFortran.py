#!/usr/bin/env python2
from __future__ import division, absolute_import

import os.path
import itertools
import unittest
import pickle

import numpy
import numpy.random
import RO.Astro.Tm

from mirrorCtrl.mirrors import mir35mSec, mir35mTert, mir25mSec, mir25mPrim
from mirrorCtrl.const import convOrient2UMArcsec, convOrient2MMRad, MMPerMicron

numpy.random.seed(0)

dataDir = os.path.join((os.path.dirname(__file__)), "data")
# these pickle files created using data/mirrorBlocks.py
# they are pickled to remove any dependencies of on the tcc.
f = open(os.path.join(dataDir, "mirDict35.p"), "rb")
mirDict35 = pickle.load(f)
f.close()
f = open(os.path.join(dataDir, "mirDict25.p"), "rb")
mirDict25 = pickle.load(f)
f.close()

Mirrors = ['prim', 'sec', 'tert']
Acts = ['A', 'B', 'C', 'D', 'E', 'F']

MaxOrientErr = numpy.array([.1, .1, .1 , .1 , .1])
MaxMountErr = 0.15 #um
MaxMountAdjNoAdjErr = 0.5 # um
# output produced from massOrient2Mount.for written by russell for mirror testing
# with fortran tcc code.
massOrientInFiles = [
    os.path.join(dataDir, "massorient_out_25_1.dat"),
    os.path.join(dataDir, 'massorient_out_25_2.dat'),
    os.path.join(dataDir, 'massorient_out_35_2.dat'),
]

class MirVsFortran(unittest.TestCase):
    def testMirrorsDotDat(self):
        self._testMirrors(useMirDat=True, maxOrientErr=MaxOrientErr, maxMountErr=MaxMountErr, maxMountAdjNoAdjErr=MaxMountAdjNoAdjErr)

    def testMirrorsMade(self):
        # allow for a little more flexibility for made mirrors
        self._testMirrors(useMirDat=False, maxOrientErr=numpy.ones(5), maxMountErr=1, maxMountAdjNoAdjErr=MaxMountAdjNoAdjErr)

    def _testMirrors(self, useMirDat, maxOrientErr, maxMountErr, maxMountAdjNoAdjErr):
        errLog = []
        for filename in massOrientInFiles:
            slurper = TheSlurper(filename, useMirDat)
            mirName = slurper.mirFile.split('_')[1].split('.')[0] # remove the .dat part
            mirNum = str(slurper.mirNum)
            adjMountDiff = numpy.abs(slurper.adjMountDiff)  # in um
            unAdjMountDiff = numpy.abs(slurper.unAdjMountDiff) # in um
            # ignore 3.5m M3, it will have large variations in adj vs un adjusted
            # if (mirName=='35m') and (mirNum=='2'): # zero indexing 2 = tertiary
            #     continue
            orientDiff = numpy.abs(slurper.orientDiff) # in um arcsec
            for ind in range(slurper.nRan):
                adjOver = True in (adjMountDiff[ind] > maxMountErr)
                unAdjOver = True in (unAdjMountDiff[ind] > maxMountAdjNoAdjErr)
                orientOver = True in (orientDiff[ind][:slurper.mirror.numAdjOrient] > maxOrientErr[:slurper.mirror.numAdjOrient])
                if True in [adjOver, unAdjOver, orientOver]:
                    #print 'failed: ', [adjOver, unAdjOver, orientOver]
                    errLog.append(
                        self.getErrMsg(
                            mirName,
                            mirNum,
                            convOrient2UMArcsec(slurper.fromOrients[ind]),
                            adjMountDiff[ind],
                            unAdjMountDiff[ind],
                            orientDiff[ind],
                        )
                    )
        if len(errLog) > 0:
            self._errLogPrint(errLog)
        self.assertEqual(len(errLog), 0, 'Errors Found and Printed To File')

    def _errLogPrint(self, errLog):
        """Print the error log to a file
        """
        #fileDate = time.localtime()
        # minutes and seconds appended to filename
        fileName = 'Errors' + RO.Astro.Tm.isoDateTimeFromPySec(nDig=0, useGMT=False) + ".log"
        with open(fileName, 'w') as f:
            f.write("\n".join(errLog))

    def getErrMsg(self, mirName, mirNum, desOrient, adjMtDiff, noAdjMtDiff, orientDiff):
        # ind index of list on the slurper from which to print info
        errStr = mirName + ' ' + mirNum
        errStr += '\tdesOrient: ' + ','.join([('%.2f'%num).zfill(6) for num in desOrient])
        errStr += '\tDiff PyAdjMount Vs FortranMount: ' + ','.join([('%.2f'%num).zfill(6) for num in adjMtDiff])
        errStr += '\tDiff UnAdjPyMount Vs AdjPyMount: ' + ','.join([('%.2f'%num).zfill(6) for num in noAdjMtDiff])
        errStr += '\tDiff PyOrient Vs FortranOrient: ' + ','.join([('%.2f'%num).zfill(6) for num in orientDiff])
        return errStr

    def testTertGeom(self):
        """Test the unusual 3.5 M3 geomery
        """
        slurper = TheSlurper(os.path.join(dataDir,'massorient_out_35_3.dat'))
        # piston the mirror 100um.  Actuators A,B,C should lengthen by
        # roughly sqrt(100**2/2)
        orient = [100. * MMPerMicron, 0, 0, 0, 0]
        mounts = numpy.array(slurper.mirror.actuatorMountFromOrient(orient, adjustOrient = False))/slurper.mountScale
        howClose = numpy.sqrt(100.**2/2.) - mounts
        for diff in howClose:
            self.assertTrue(numpy.abs(diff) < 1.) # within 1 micron

class TheSlurper(object):
    """An object for generating data on orient to mount conversions
    and comparing with the FORTRAN conversion code
    """
    def __init__(self, massorientfile, useMirDat=True):
        """Take a massorientfile, build everything, run conversions

        @param[in] massorientfile  a data/massorient_out*.dat
        @param[in] useMirDat  boolean. construct a mirror from a mir.dat file (rather than a measured mirror, with separate encoder positions...)

        These are the output of Russell's massorient2mount.for script. I have manually
        appended the correstponding data/mir*.dat file and the mirror number on the
        first two lines.
        """
        parsed = self.parseMassOrient(massorientfile)
        attrs = ['mirFile', 'mirNum', 'fromOrients', 'mountsFOR', 'toOrientsFOR']
        for attr, data in itertools.izip(attrs, parsed):
            setattr(self, attr, data)
        if self.mirFile == 'mir_25m.dat':
            if useMirDat:
                self.mirror = mirDict25[Mirrors[self.mirNum]]
            else:
                self.mirror = mir25mSec if self.mirNum==1 else mir25mPrim
        elif self.mirFile == 'mir_35m.dat':
            if useMirDat:
                self.mirror = mirDict35[Mirrors[self.mirNum]]
            else:
                self.mirror = mir35mTert if self.mirNum==2 else mir35mSec
        else:
            raise RuntimeError('Unrecognized mirror file')
        # clean up mounts from massorient, remove trailing zeros for non movable axes
        self.nAct = len(self.mirror.actuatorList)
        self.mountsFOR = [mount[:self.nAct] for mount in self.mountsFOR]
        self.mountsPY, self.mountsPYnoAdj = self.orients2mounts()
        self.orientsPY = self.mounts2orients()
        self.mountScale = numpy.asarray([link.scale for link in self.mirror.actuatorList]) # in mount units / um
        self.nRan = len(self.mountsFOR)

    @property
    def adjMountDiff(self): # in um
        #difference between fortran mount lengths and python mount lengths from same orientation
        # returns an n x nAct array
        return (self.mountsFOR - self.mountsPY) / numpy.tile(self.mountScale, (self.nRan, 1))

    @property
    def unAdjMountDiff(self): # in um
        #mount length difference computed from adjusted orientation, and unadjusted orientation
        # returns an n x nAct array
        return (self.mountsPY - self.mountsPYnoAdj) / numpy.tile(self.mountScale, (self.nRan, 1))

    @property
    def orientDiff(self): # in um and arcseconds
        # difference between Fortran computed orientation and python computed orientation, from the same mount coordinates
        # note these should differ because python includes induced motions from fixed links.
        return convOrient2UMArcsec(self.toOrientsFOR - self.orientsPY)

    def parseMassOrient(self, massorientfile):
        """return mirror info and orientation/mount data from a massorient file"""
        with open(massorientfile, 'r') as f:
            lines = f.readlines()
        mirFile = str(lines.pop(0).strip())
        mirNum = int(lines.pop(0).strip()) - 1 # zero indexing
        fromOrients = []
        mounts = []
        toOrients = []
        indRange = numpy.arange(len(lines))
        numpy.random.shuffle(indRange)
        for ind in indRange[:20]:
        #for line in lines:
            nums = lines[ind].split()
            nums = [float(num) for num in nums]
            fromOrients.append(convOrient2MMRad(nums[:6])) # to mm and radians
            mounts.append(numpy.asarray(nums[6:12]))
            toOrients.append(convOrient2MMRad(nums[12:])) # to mm and radians
        return mirFile, mirNum, numpy.asarray(fromOrients), numpy.asarray(mounts), numpy.asarray(toOrients)

    def orients2mounts(self):
        mountsPYadj = []
        mountsPYnoAdj = []
        for orient in self.fromOrients:
            try:
                mountAdj = self.mirror.actuatorMountFromOrient(orient)
                mountNoAdj = self.mirror.actuatorMountFromOrient(orient, adjustOrient = False)
            except RuntimeError:
                print 'out of range?'
                mountAdj = numpy.zeros(self.nAct)*numpy.nan
                mountNoAdj = mountAdj[:]
            mountsPYadj.append(mountAdj)
            mountsPYnoAdj.append(mountNoAdj)
        return numpy.asarray(mountsPYadj), numpy.asarray(mountsPYnoAdj)

    def mounts2orients(self):
        orientsPY = []
        for mount in self.mountsFOR:
            try:
                orient = self.mirror.orientFromActuatorMount(mount)
            except RuntimeError:
                print 'did not converge'
                orient = numpy.zeros(6)*numpy.nan
            orientsPY.append(orient)
        return numpy.asarray(orientsPY)

if __name__ == "__main__":
    unittest.main()
