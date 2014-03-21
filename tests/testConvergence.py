#!/usr/bin/env python
import itertools
import copy
import math
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

from mirrorCtrl.const import convOrient2MMRad, MMPerMicron, RadPerArcSec
from mirrorCtrl.mirrors import mir35mSec, mir35mTert, mir25mSec
from testMirrorCtrl import  CmdCallback #getOpenPort #, UserPort, MirrorCtrlTestBase,
from mirrorCtrl import FakeDispatcherWrapper, GalilDevice
import mirrorCtrl.fakeGalil

pwd = os.path.dirname(__file__)
secMoveList = pickle.load(open(os.path.join(pwd, "data/secMoveList.p")))
tertMoveList = pickle.load(open(os.path.join(pwd, "data/tertMoveList.p")))

mirrorCtrl.galilDevice.MaxIter = 12
mirrorCtrl.fakeGalil.MaxCmdTime = 0.025

RussellsOrient = numpy.asarray([834.26, -24.03, 366.27, -192.64, 1388.21]) # in um, arcsec
randInds = numpy.random.randint(0, len(secMoveList), 2)# a few random indices from real mirror positions from log
m2TestOrients = [numpy.asarray(d["desOrient"]) for d in [secMoveList[ind] for ind in randInds]]
m2TestOrients.append(RussellsOrient)
m3TestOrients = [numpy.asarray(d["desOrient"]) for d in [tertMoveList[ind] for ind in randInds]]

LargePiston, LargeTilt, LargeTranslation = GalilDevice.LargePiston, GalilDevice.LargeTilt, GalilDevice.LargeTranslation

def getActEqEncMir(mirror):
    """ Returns the same mirror as input, except actuators are moved to be exactly aligned with actuators
    @param[in] mirror: a MirrorBase instance
    @return a mirror instance with moved actuators
    """
    mirror = copy.deepcopy(mirror)
    for act, enc in itertools.izip(mirror.actuatorList, mirror.encoderList):
        act.mirPos = enc.mirPos[:]
        act.basePos = enc.basePos[:]
    return mirror

def getActRandMove(mirror, seed=10):
    """ Apply a random xy translation to ABC actuators,
        Apply a random z translation to DE actuators
    """
    enc0BasePos = mir35mSec.encoderList[0].basePos
    encRad = math.hypot(enc0BasePos[0], enc0BasePos[1])
    act0BasePos = mir35mSec.actuatorList[0].basePos
    actRad = math.hypot(act0BasePos[0], act0BasePos[1])
    numpy.random.seed(seed)
    mirror = copy.deepcopy(mirror)
    lengthScale = numpy.abs(actRad - encRad)*2.
    for act in mirror.actuatorList[:3]:
        # let offsets vary by magnitude of true offset in any direction
        xOff, yOff = numpy.random.sample(2)*2.*lengthScale - lengthScale
        offset = numpy.asarray([xOff, yOff, 0.])
        act.mirPos += offset
        act.basePos += offset
    for act in mirror.actuatorList[3:]:
        # let offset vary by magnitude of true offset in z direction
        zOff = numpy.random.sample()*2.*lengthScale*0.5 - lengthScale*0.5
        offset = numpy.asarray([0., 0., zOff])
        act.mirPos += offset
        act.basePos += offset
    return mirror


class MirState(object):
    def __init__(self, mirror):
        """ For keeping track of a mirror orietation.
            @param[in] mirror: a MirrorBase instance
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
        self.dw = FakeDispatcherWrapper(
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
        """ @param[in] modelMirrorState: MirState object representing an imperfect model
            @param[in] trueMirState: MirState object representing the true mirror
            @param[in] desOrient: a collection of 5 items: pistion, tiltx, tilty, transx, transy. units um and arcsec
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
        self.fakeGalilFactory: fake Galil factory, e.g. FakeGalilFactory or FakePiezoGalilFactory
        self.mirror: mirorCtrl Mirror, e.g. mirrorCtrl.mirrors.mir35mTert
        self.mirDev: mirror device, e.g. mirrorCtrl.GalilDevice
        self.name: name of keyword dict for this mirror
        """
        raise NotImplementedError()

    def _testOrient(self, orient):
        """ @param[in] orientation: 5 element orientation in user-friendly units (um and arcsec)
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
            self.assertTrue(self.dispatcher.model.iter.valueList[0] <= nIter)
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

class ConvergenceTestActEqEnc(ConvergenceTestBase, TestCase):
    def setVars(self):
        # self.userPort = getOpenPort()
        # self.fakeGalilFactory = FakeGalil
        self.trueMirror = mir35mSec
        self.mirror = getActEqEncMir(self.trueMirror)
        # self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m2TestOrients)

    def testSmallMoves(self):
        bigOrient = numpy.asarray([LargePiston/MMPerMicron, LargeTilt/RadPerArcSec, LargeTilt/RadPerArcSec, LargeTranslation/MMPerMicron, LargeTranslation/MMPerMicron], dtype=float)
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        orient2 = orient1 + 0.5*bigOrient
        orient3 = orient2 + 0.75*bigOrient
        return self._testOrients([orient1, orient2, orient3]) # should automatically add offsets

    def testSameMoves(self):
        # offset should be pre-determined and automatic for moves 2 and 3
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        return self._testOrients([orient1]*3) # should automatically add offsets

    def testBigMoves(self):
        bigOrient = numpy.asarray([LargePiston/MMPerMicron, LargeTilt/RadPerArcSec, LargeTilt/RadPerArcSec, LargeTranslation/MMPerMicron, LargeTranslation/MMPerMicron], dtype=float)
        orient1 = numpy.asarray(m2TestOrients[0], dtype=float)
        orient2 = orient1 + 1.5*bigOrient
        orient3 = orient2 + 2.75*bigOrient
        return self._testOrients([orient1, orient2, orient3])      # no added offset

class ConvergenceTestM3(ConvergenceTestBase, TestCase):
    def setVars(self):
        # self.userPort = getOpenPort()
        # self.fakeGalilFactory = FakeGalil
        self.trueMirror = mir35mTert
        self.mirror = getActEqEncMir(self.trueMirror)
        # self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m3TestOrients)

class ConvergenceTestSDSSM2(ConvergenceTestBase, TestCase):
    def setVars(self):
        # self.userPort = getOpenPort()
        # self.fakeGalilFactory = FakePiezoGalil
        self.trueMirror = mir25mSec
        self.mirror = getActEqEncMir(self.trueMirror)
        # self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror"

    def testOrients(self):
        orientList = [
            [2000, 20, 20, 4, 8],
            [-2000, -40, 89, 45, -23],
        ]
        return self._testOrients(orientList)

class ConvergenceTestRandAct(ConvergenceTestBase, TestCase):
    def setVars(self):
        # self.userPort = getOpenPort()
        # self.fakeGalilFactory = FakeGalil
        self.trueMirror = mir35mSec
        self.mirror = getActRandMove(self.trueMirror, seed=45)
        # self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror"

    def testOrients(self):
        return self._testOrients(m2TestOrients)

class ConvergenceTestPerfect(ConvergenceTestBase, TestCase):
    """model exactly represents truth, no iterations
    """
    def setVars(self):
        # self.userPort = getOpenPort()
        # self.fakeGalilFactory = FakeGalil
        self.trueMirror = mir35mSec
        self.mirror = copy.deepcopy(mir35mSec)
        # self.mirDev = mirrorCtrl.GalilDevice
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
