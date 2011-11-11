#!/usr/bin/env python
import unittest
import numpy

import parseMirFile_adjBase
import parseMirFile_adjLen
import mirror

# construct range of orients to test (0-25mm, 0-.1 radians)
resolution = 10
dist_range = numpy.linspace(0, 25, resolution)
ang_range = numpy.linspace(0, .1, resolution)
ranges = [dist_range, ang_range, ang_range, dist_range, dist_range]
orientRange = numpy.zeros((resolution * 5, 5))
for ind, rng in enumerate(ranges):
    orientRange[ind*resolution:ind*resolution+resolution, ind] = rng

class prim_25m(unittest.TestCase):

    def test_25m_prim(self):
        actList = parseMirFile_adjLen.mir25_prim_actList
        #encList = parseMirFile.mir25_prim_encList
        dirMir = mirror.DirectMirror(actList, [])
        for orientTest in orientRange:
            mount = dirMir.actuatorMountFromOrient(orientTest)
            orient1 = dirMir.orientFromActuatorMount(mount)
            mount1 = dirMir.actuatorMountFromOrient(orient1)
            # commanded orient is only 5 axes, add a zero for rotation for math to work
            orientTest = numpy.hstack((numpy.array(orientTest),0.))
            orientErr = orientTest - orient1
            mountErr = numpy.asarray(mount, dtype=float) - numpy.asarray(mount1, dtype=float)
            for oE in orientErr:
                self.assertAlmostEqual(oE, 0., places=4)
            for mE in mountErr:
                self.assertAlmostEqual(mE, 0., places=4)








if __name__ == '__main__':
    unittest.main()