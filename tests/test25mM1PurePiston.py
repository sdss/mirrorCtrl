#!/usr/bin/env python2
from __future__ import division, absolute_import

import itertools
import copy

from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred

from twistedActor import UserCmd, testUtils
testUtils.init(__file__)

from mirrorCtrl.mirrors import mir25mPrim
from mirrorCtrl import GalilDeviceWrapper
from mirrorCtrl.const import convOrient2MMRad

# orientation from log.  Commanded twice saw non pistion only offset:
# 2015-08-22 05:20:23.599 info  UserCmd('332 move -1483.3273,-24.2100,6.5200,356.1000,1323.0000', state=done)
orientation =  [-1483.3273, -24.2100, 6.5200, 356.1000, 1323.0000, 0.0]
orientMMRad = convOrient2MMRad(orientation)
pistonOrient = copy.deepcopy(orientMMRad)
pistonOrient[0] = pistonOrient[0]*1.05 # 1 percent change in piston

class Test25mM1PurePiston(TestCase):

    def setUp(self):
        self.dw = GalilDeviceWrapper(
            mirror=mir25mPrim,
        )
        return self.dw.readyDeferred

    def tearDown(self):
        d = self.dw.close()
        return d

    @property
    def galilDevice(self):
        return self.dw.device

    def testSameOrient(self):
        # Dan L noticed non pure pistion offsets if same orient is commanded?
        userCmd1 = UserCmd()
        userCmd2 = UserCmd()
        cmdMounts = []
        modelMounts = []
        d = Deferred()
        def setDefDone(userCmd2):
            if userCmd2.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount))
                for mount1, mount2 in itertools.izip(cmdMounts[0], cmdMounts[1]):
                    self.assertEqual(mount1, mount2)
                for mount1, mount2 in itertools.izip(modelMounts[0], modelMounts[1]):
                    self.assertEqual(mount1, mount2)
                d.callback("Done")
        def sendOrientAgain(userCmd1):
            if userCmd1.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount))
                #explicitly check computations
                self.assertTrue(self.galilDevice.isSameOrientation(orientMMRad))
                self.assertFalse(self.galilDevice.isPistonOnly(orientMMRad))
                #really send the command and check state afterwards
                self.galilDevice.cmdMove(userCmd2, orientMMRad)
        userCmd1.addCallback(sendOrientAgain)
        userCmd2.addCallback(setDefDone)
        self.galilDevice.cmdMove(userCmd1, orientMMRad)
        return d

    def testPistonChange(self):
        # Dan L noticed non pure pistion offsets if same orient is commanded?
        userCmd1 = UserCmd()
        userCmd2 = UserCmd()
        cmdMounts = []
        modelMounts = []
        d = Deferred()
        def setDefDone(userCmd2):
            if userCmd2.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount))
                i=0
                for mount1, mount2 in itertools.izip(cmdMounts[0], cmdMounts[1]):
                    if i <= 2:
                        self.assertNotEqual(mount1, mount2)
                    else:
                        self.assertEqual(mount1, mount2)
                    i+=1
                i=0
                for mount1, mount2 in itertools.izip(modelMounts[0], modelMounts[1]):
                    if i <= 2:
                        self.assertNotEqual(mount1, mount2)
                    else:
                        self.assertEqual(mount1, mount2)
                    i+=1
                d.callback("Done")
        def sendNewPiston(userCmd1):
            if userCmd1.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount)) # why the deep copy needed?, trust me it is needed
                #explicitly check computations
                self.assertFalse(self.galilDevice.isSameOrientation(pistonOrient))
                self.assertTrue(self.galilDevice.isPistonOnly(pistonOrient))
                #really send the command and check state afterwards
                self.galilDevice.cmdMove(userCmd2, pistonOrient)
        userCmd1.addCallback(sendNewPiston)
        userCmd2.addCallback(setDefDone)
        self.galilDevice.cmdMove(userCmd1, orientMMRad)
        return d

    def testPistonThenSame(self):
        userCmd1 = UserCmd()
        userCmd2 = UserCmd()
        userCmd3 = UserCmd()
        cmdMounts = []
        modelMounts = []
        d = Deferred()
        def sendNewPiston(userCmd1):
            if userCmd1.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount)) # why the deep copy needed?, trust me it is needed
                #explicitly check computations
                self.assertFalse(self.galilDevice.isSameOrientation(pistonOrient))
                self.assertTrue(self.galilDevice.isPistonOnly(pistonOrient))
                #really send the command and check state afterwards
                self.galilDevice.cmdMove(userCmd2, pistonOrient)
        def sendNewPistonAgain(userCmd2):
            if userCmd2.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount))
                i=0
                for mount1, mount2 in itertools.izip(cmdMounts[0], cmdMounts[1]):
                    if i <= 2:
                        self.assertNotEqual(mount1, mount2)
                    else:
                        self.assertEqual(mount1, mount2)
                    i+=1
                i=0
                for mount1, mount2 in itertools.izip(modelMounts[0], modelMounts[1]):
                    if i <= 2:
                        self.assertNotEqual(mount1, mount2)
                    else:
                        self.assertEqual(mount1, mount2)
                    i+=1
                self.galilDevice.cmdMove(userCmd3, pistonOrient)
        def setDefDone(userCmd3):
            if userCmd3.isDone:
                cmdMounts.append(self.galilDevice.status.cmdMount)
                modelMounts.append(copy.deepcopy(self.galilDevice.status.modelMount))
                for mount1, mount2 in itertools.izip(cmdMounts[1], cmdMounts[2]):
                    self.assertEqual(mount1, mount2)
                for mount1, mount2 in itertools.izip(modelMounts[1], modelMounts[2]):
                    self.assertEqual(mount1, mount2)
                d.callback("Done")
        userCmd1.addCallback(sendNewPiston)
        userCmd2.addCallback(sendNewPistonAgain)
        userCmd3.addCallback(setDefDone)
        self.galilDevice.cmdMove(userCmd1, orientMMRad)
        return d

if __name__ == "__main__":
    from unittest import main
    main()
