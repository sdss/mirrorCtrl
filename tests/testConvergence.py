#!/usr/bin/env python2
from __future__ import division, absolute_import

import os
import pickle

import numpy
import numpy.random
numpy.random.seed(10)
import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from opscore.actor import CmdVar
from twisted.internet.defer import Deferred
from twisted.trial.unittest import TestCase
from twistedActor import testUtils

testUtils.init(__file__)

from mirrorCtrl.const import convOrient2MMRad, MMPerMicron, RadPerArcSec
from mirrorCtrl.perturbActuators import getActEqEncMir, getActRandMove
from mirrorCtrl.mirrors import mir35mSec, mir35mTert, mir25mSec
from testMirrorCtrl import  CmdCallback #getOpenPort #, UserPort, MirrorCtrlTestBase,
from mirrorCtrl import MirrorDispatcherWrapper
import mirrorCtrl.fakeGalil

pwd = os.path.dirname(__file__)
secMoveList = pickle.load(open(os.path.join(pwd, "data/secMoveList.p")))
tertMoveList = pickle.load(open(os.path.join(pwd, "data/tertMoveList.p")))

mirrorCtrl.galilDevice.MaxIter = 12
mirrorCtrl.fakeGalil.MaxCmdTime = 0.025
mirrorCtrl.fakeGalil.ADDNOISE = False # add no noise to encoder reading, assume perfect

RussellsOrient = numpy.asarray([834.26, -24.03, 366.27, -192.64, 1388.21]) # in um, arcsec
randInds = numpy.random.randint(0, len(secMoveList), 2)# a few random indices from real mirror positions from log
m2TestOrients = [numpy.asarray(d["desOrient"]) for d in [secMoveList[ind] for ind in randInds]]
m2TestOrients.append(RussellsOrient)
m3TestOrients = [numpy.asarray(d["desOrient"]) for d in [tertMoveList[ind] for ind in randInds]]

LargePiston, LargeTilt, LargeTranslation = 500. * MMPerMicron, 10. * RadPerArcSec, 100. * MMPerMicron

class MirState(object):
    def __init__(self, mirror):
        """ For keeping track of a mirror orietation.
            @param[in] mirror  a MirrorBase instance
        """
        self.mirror = mirror
        self.orientation = numpy.zeros(5)
        self.actPos = self.mirror.actuatorMountFromOrient(self.orientation)
        self.encPos = self.mirror.encoderMountFromOrient(self.orientation)
        self.mountOffset = numpy.zeros(len(self.actPos))

    def moveToActPos(self, actPos):
        self.actPos = actPos + self.mountOffset# actuators go where they were told + any offset defined
        self.orientation = self.mirror.orientFromActuatorMount(actPos) # orientation is found from where the actuators went
        self.encPos = self.mirror.encoderMountFromOrient(self.orientation) # encoder position reflects the true orientation

    def setMountOffset(self, offset):
        self.mountOffset = offset

class ConvergenceTestBase(object):
    def setUp(self):
        self.setVars()
        self.dw = MirrorDispatcherWrapper(
            mirror=self.trueMirror,
        )
        def overwriteMirror(foo=None):
            assert(self.dw.actorWrapper.deviceWrapperList[0].device.mirror is not None)
            self.dw.actorWrapper.deviceWrapperList[0].device.mirror = self.mirror
        self.dw.readyDeferred.addCallback(overwriteMirror)
        return self.dw.readyDeferred

    def tearDown(self):
        from twisted.internet import reactor
        delayedCalls = reactor.getDelayedCalls()
        for call in delayedCalls:
            call.cancel()
        d = self.dw.close()
        return d

    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher

    @property
    def mirDev(self):
        """return the device talking to the fake galil
        """
        self.dw.actorWrapper.deviceWrapperList[0].device

    def _testConv(self, modelMirState, trueMirState, desOrient):
        """ @param[in] modelMirrorState  MirState object representing an imperfect model
            @param[in] trueMirState  MirState object representing the true mirror
            @param[in] desOrient  a collection of 5 items: pistion, tiltx, tilty, transx, transy. units um and arcsec
        """
        desOrient = convOrient2MMRad(desOrient)
        cmdActPos = numpy.asarray(modelMirState.mirror.actuatorMountFromOrient(desOrient)) # get commanded actuator position from your model
        newCmdActPos = cmdActPos[:]
        trueMirState.moveToActPos(cmdActPos) # set the true mirror's actuator lengths
        for iters in range(1,10):
            trueOrient = numpy.asarray(trueMirState.orientation[0:5])
            measActPos = numpy.asarray(modelMirState.mirror.actuatorMountFromOrient(trueOrient))
            actDiff = cmdActPos - measActPos
            # check if actuator error is small enough to exit
            if numpy.all(numpy.abs(actDiff)/trueMirState.mirror.minCorrList < 1.):
                break
            newCmdActPos = actDiff*0.9 + newCmdActPos
            trueMirState.moveToActPos(newCmdActPos)
        finalOffset = cmdActPos - newCmdActPos
        return iters, finalOffset

    def setVars(self):
        """overwritten by subclasses

        must set the following instance variables (shown by example):
        self.fakeGalilFactory: fake Galil factory, e.g. FakeGalilFactory
        self.mirror: mirorCtrl Mirror, e.g. mirrorCtrl.mirrors.mir35mTert
        self.mirDev: mirror device, e.g. mirrorCtrl.GalilDevice
        self.name: name of keyword dict for this mirror
        """
        raise NotImplementedError()

    def _testOrient(self, orient):
        """ @param[in] orientation  5 element orientation in user-friendly units (um and arcsec)
        """
        # print 'previous orient ', prevOrient
        # print 'testing orient', orient
        trueMirState = MirState(self.trueMirror)
        modelMirState = MirState(self.mirror)
        nIter, finalOffset = self._testConv(modelMirState, trueMirState,  orient)
        d = Deferred()
        cmdStr = 'move ' + ', '.join([str(x) for x in orient])
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            # reason for less than or equal:
            # if new orient is a big difference from the previous, an automatic offset
            # should have been applied, thus taking less iterations to converge
            self.assertTrue(self.dispatcher.model.state.valueList[1] <= nIter)
        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def _testOrients(self, orients):
        """test a set of orientations
        """
        iterOs = iter(orients)
        outerD = Deferred()
        def doNext(foo='for a callback'):
            try:
                nextOrient = iterOs.next()
                innerD = self._testOrient(nextOrient)
                innerD.addCallback(doNext)
            except StopIteration:
                # done with all
                outerD.callback("go")
        doNext()
        return outerD

class TestIteration(ConvergenceTestBase, TestCase):
    def setVars(self):
        self.trueMirror = mir35mSec
        self.mirror = mir35mSec # set equal because the fake galil autimatically sets act=enc
        self.name = "mirror"

    def testIteration(self):
        # print 'previous orient ', prevOrient
        # print 'testing orient', orient
        orient = RussellsOrient[:]
        # trueMirState = MirState(self.trueMirror)
        # modelMirState = MirState(self.mirror)
        # nIter, finalOffset = self._testConv(modelMirState, trueMirState,  orient)
        d = Deferred()
        cmdStr = 'move ' + ', '.join([str(x) for x in orient])
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            print 'iterlist', self.dispatcher.model.state.valueList[1]
            print 'netMountOffset', self.dispatcher.model.netMountOffset.valueList[0]
            self.assertTrue(self.dispatcher.model.state.valueList[1] > 1)
        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

class ConvergenceTestActEqEnc(ConvergenceTestBase, TestCase):
    def setVars(self):
        self.trueMirror = mir35mSec
        self.mirror = getActEqEncMir(self.trueMirror)
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m2TestOrients)

    def testSmallMoves(self):
        bigOrient = numpy.asarray([LargePiston/MMPerMicron, LargeTilt/RadPerArcSec, LargeTilt/RadPerArcSec, LargeTranslation/MMPerMicron, LargeTranslation/MMPerMicron], dtype=float)
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        orient2 = orient1 + 0.5*bigOrient
        orient3 = orient2 + 0.75*bigOrient
        return self._testOrients([orient1, orient2, orient3])

    def testSameMoves(self):
        # offset should be pre-determined and automatic for moves 2 and 3
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        return self._testOrients([orient1]*3) # should automatically add offsets

    def testBigMoves(self):
        bigOrient = numpy.asarray([LargePiston/MMPerMicron, LargeTilt/RadPerArcSec, LargeTilt/RadPerArcSec, LargeTranslation/MMPerMicron, LargeTranslation/MMPerMicron], dtype=float)
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        orient2 = orient1 + 1.5*bigOrient
        orient3 = orient2 + 2.75*bigOrient
        return self._testOrients([orient1, orient2, orient3])

class ConvergenceTestM3(ConvergenceTestBase, TestCase):
    def setVars(self):
        self.trueMirror = mir35mTert
        self.mirror = getActEqEncMir(self.trueMirror)
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m3TestOrients)

class ConvergenceTestSDSSM2(ConvergenceTestBase, TestCase):
    def setVars(self):
        self.trueMirror = mir25mSec
        self.mirror = getActEqEncMir(self.trueMirror)
        self.name = "mirror"

    def testOrients(self):
        orientList = [
            [2000, 20, 20, 4, 8],
            [-2000, -40, 89, 45, -23],
        ]
        return self._testOrients(orientList)

class ConvergenceTestRandAct(ConvergenceTestBase, TestCase):
    def setVars(self):
        self.trueMirror = mir35mSec
        self.mirror = getActRandMove(self.trueMirror, seed=45)
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m2TestOrients)

class ConvergenceTestPerfect(ConvergenceTestBase, TestCase):
    """model exactly represents truth, no iterations
    """
    def setVars(self):
        self.trueMirror = mir35mSec
        self.mirror = getActEqEncMir(mir35mSec) # fake galil uses a non-perfect model
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m2TestOrients)

    def _testConv(self, modelMirState, trueMirState, desOrient):
        iters = 1
        offset = 'foo'
        return iters, offset


if __name__ == "__main__":
    from unittest import main
    main()
