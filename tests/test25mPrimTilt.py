#!/usr/bin/env python2
from __future__ import division, absolute_import
"""http://twistedmatrix.com/documents/current/core/howto/trial.html

Tests communication and other behavior between the Actor and Device. Commands are dispatched using a
dispatcher.
"""

import copy
import itertools

import numpy

from twisted.trial.unittest import TestCase
from twisted.internet.defer import gatherResults, Deferred
# from twisted.internet import reactor
from twistedActor import testUtils, UserCmd #, log

testUtils.init(__file__)

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
# from RO.Comm.TwistedTimer import Timer
# from opscore.actor import CmdVar

from mirrorCtrl import GalilDeviceWrapper
from mirrorCtrl.mirrors import mir25mPrim as mir25mPrimNew
from mirrorCtrl.const import convOrient2MMRad, RadPerArcSec,convOrient2UMArcsec, MMPerMicron

# set up a mirror without the tilts included in offsets
mir25mPrimOld = copy.deepcopy(mir25mPrimNew)
trueOffsets = [9700., 800., 5650., -1650., -6900., -6900]
oldOffsets = [10850., -450., 5750., -1650., -6900., -6900] # these have hidden tilts
for act, oldOffset in itertools.izip(mir25mPrimOld.actuatorList, oldOffsets):
    act.offset = oldOffset
for act, newOffset in itertools.izip(mir25mPrimNew.actuatorList, trueOffsets):
    # explicitly set these incase they change in the future, this test will still pass
    act.offset = newOffset
#kaikes hidden tilts:
# xtilt = -17.2
# ytilt = 11.9
#conor solved hidden tilts:
# xtilt = -16.45
# ytilt = 7.7
# scipy optimizer solved
xtiltNew=-17.47
ytiltNew=11.84
xtransNew=635.9
ytransNew=916.8


# # orientatiNewons commanded from prim25m.log-20150429
# orientations = [
#     [-2455.0205,0.0000,0.0000,651.0000,937.4000],
#     [-2003.8499,0.0000,0.0000,651.0000,937.4000],
#     [-1992.2187,0.0000,0.0000,651.0000,937.4000],
#     [-1972.4714,0.0000,0.0000,651.0000,937.4000],
# ]
# # convert orientations to arcsecs/rad
# orientations = [convOrient2MMRad(orient) for orient in orientations]

class CmdCallback(object):
    """Call a deferred when a command is finished
    """
    def __init__(self, deferred):
        self.deferred = deferred

    def __call__(self, cmd):
        # print "generic command callback: %r, %s" % (cmd, cmd.lastCode)
        if cmd.isDone:
            deferred, self.deferred = self.deferred, None
            deferred.callback("done")

class Test25mPrimTilt(TestCase):
    def setUp(self):
# orientations commanded from prim25m.log-20150429
        orientations = [
            [-2455.0205,0.0000,0.0000,651.0000,937.4000],
            [-2003.8499,0.0000,0.0000,651.0000,937.4000],
            [-1992.2187,0.0000,0.0000,651.0000,937.4000],
            [-1972.4714,0.0000,0.0000,651.0000,937.4000],
        ]
        # convert orientations to arcsecs/rad
        self.orientations = [convOrient2MMRad(orient) for orient in orientations]
        self.hiddenTilts = GalilDeviceWrapper(
            mirror=mir25mPrimOld,
        )
        self.noHiddenTilts = GalilDeviceWrapper(
            mirror=mir25mPrimNew,
        )
        return gatherResults([self.hiddenTilts.readyDeferred, self.noHiddenTilts.readyDeferred])

    def tearDown(self):
        return gatherResults([self.hiddenTilts.close(), self.noHiddenTilts.close()])

    @property
    def galilDeviceTilt(self):
        return self.hiddenTilts.device

    @property
    def galilDeviceNoTilt(self):
        return self.noHiddenTilts.device

    def cmdMove(self, galilDevice, orientation):
        cmd = UserCmd()
        galilDevice.cmdMove(cmd, orientation)
        return cmd

    def _testPurePistion(self, orientation1, orientation2, noHiddenTilts=False):
        d = Deferred()
        galilDevice = self.galilDeviceNoTilt if noHiddenTilts else self.galilDeviceTilt
        cmd1 = self.cmdMove(galilDevice, orientation1)
        def nextMove(cmd):
            if cmd.isDone:
                # make sure this is a piston only move
                self.assertTrue(galilDevice.isPistonOnly(orientation2))
                cmd2 = self.cmdMove(galilDevice, orientation2)
                cmd2.addCallback(CmdCallback(d))
        cmd1.addCallback(nextMove)
        return d

    def testPurePiston01(self):
        return self._testPurePistion(self.orientations[0], self.orientations[1])

    def testPurePiston02(self):
        return self._testPurePistion(self.orientations[0], self.orientations[2])

    def testPurePiston03(self):
        return self._testPurePistion(self.orientations[0], self.orientations[3])

    def testPurePiston04(self):
        return self._testPurePistion(self.orientations[3], self.orientations[1])

    def testPurePiston05(self):
        return self._testPurePistion(self.orientations[2], self.orientations[3])

    def testPurePiston06(self):
        orientation1 = self.orientations[2][:]
        orientation2 = self.orientations[1][:]
        for orient in [orientation1, orientation2]:
            orient[1] = -17.2*RadPerArcSec
            orient[2] = 11.9*RadPerArcSec
        return self._testPurePistion(orientation1, orientation2)

    def testPurePiston07(self):
        orientation1 = self.orientations[0][:]
        orientation2 = self.orientations[1][:]
        for orient in [orientation1, orientation2]:
            orient[1] = -27.2*RadPerArcSec
            orient[2] = 4.9*RadPerArcSec
        return self._testPurePistion(orientation1, orientation2)

    def testPurePiston08(self):
        orientation1 = self.orientations[0][:]
        orientation2 = self.orientations[2][:]
        for orient in [orientation1, orientation2]:
            orient[1] = 17.2*RadPerArcSec
            orient[2] = -8.1*RadPerArcSec
        return self._testPurePistion(orientation1, orientation2)

    def testPurePiston09(self):
        orientation1 = self.orientations[0][:]
        orientation2 = self.orientations[3][:]
        for orient in [orientation1, orientation2]:
            orient[1] = 2.34*RadPerArcSec
            orient[2] = -21.9*RadPerArcSec
        return self._testPurePistion(orientation1, orientation2)

    def testPurePiston10(self):
        orientation1 = self.orientations[0][:]
        orientation2 = self.orientations[2][:]
        for orient in [orientation1, orientation2]:
            orient[1] = 17.2*RadPerArcSec
            orient[2] = -8.1*RadPerArcSec
        return self._testPurePistion(orientation1, orientation2, noHiddenTilts=True)

    def testPurePiston11(self):
        return self._testPurePistion(self.orientations[2], self.orientations[3], noHiddenTilts=True)

    def testOffsetVsCommandedTilts(self):
        for i in range(len(self.orientations)):
            orientation1 = self.orientations[i][:]
            orientation2 = [orientation1[0]] + [xtiltNew*RadPerArcSec, ytiltNew*RadPerArcSec, xtransNew*MMPerMicron, ytransNew*MMPerMicron]
            # orientation2 = copy.deepcopy(orientation1[:])
            # orientation2[1] = xtilt*RadPerArcSec
            # orientation2[2] = ytilt*RadPerArcSec
            # print("orientation 1", convOrient2UMArcsec(orientation1))
            # print("orientation 2", convOrient2UMArcsec(orientation2))
            mount1, adjOrient1 = self.galilDeviceTilt.mirror.actuatorMountFromOrient(orientation1, return_adjOrient = True, adjustOrient = True)
            # print("cmdMount1 ", mount1)
            # print("adjOrient1 ", adjOrient1)
            mount2, adjOrient2 = self.galilDeviceNoTilt.mirror.actuatorMountFromOrient(orientation2, return_adjOrient = True, adjustOrient = True)
            # print("cmdMount2 ", mount2)
            # print("adjOrient2 ", adjOrient2)
            # print("")
            # print("mount Err: %s"%", ".join([str(x) for x in numpy.subtract(mount1, mount2)]))
            # print("")
            # print("")
            self.assertTrue(numpy.all(numpy.abs(numpy.subtract(mount1, mount2))<50))


if __name__ == '__main__':
    from unittest import main
    main()
