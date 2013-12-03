#!/usr/bin/env python
from mirrorCtrl.mirrors import mir35mSec
import numpy
import itertools
import copy
import math
import numpy.random
numpy.random.seed(10)
#from data.mirLogParse import secMoveList
import pickle
import os

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from mirrorCtrl.fakeGalil import FakeGalilFactory
from testMirrorCtrl import MirrorCtrlTestBase, CmdCallback
from twisted.internet import reactor
from twisted.internet.defer import Deferred
import mirrorCtrl
from opscore.actor import CmdVar

pwd = os.path.dirname(__file__)
secMoveList = pickle.load(open(os.path.join(pwd, "data/secMoveList.p")))
# set max iter to 12
mirrorCtrl.galilDevice.MaxIter = 12

MMPerMicron = 1. / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.asarray([MMPerMicron, RadPerArcSec, RadPerArcSec, MMPerMicron, MMPerMicron])
RussellsOrient = numpy.asarray([834.26, -24.03, 366.27, -192.64, 1388.21]) # in um, arcsec
randInds = numpy.random.randint(0, len(secMoveList), 2)# a few random indices from real mirror positions from log
m2TestOrients = [numpy.asarray(d["desOrient"]) for d in [secMoveList[ind] for ind in randInds]]
m2TestOrients.append(RussellsOrient)
#m2TesetOrients = secMoveList + [RussellsOrient]

#MaxOrientErr = numpy.asarray([1, .01, .01, 1, 1])*ConvertOrient


TrueMirror = mir35mSec.Mirror
#trueMirror.plotMirror()

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
    from mirrorCtrl.mirrors.mir35mSec import actRad, encRad
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
        # zOff = numpy.random.sample()*2.*zTrue - zTrue
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

def testConv(modelMirState, trueMirState, desOrient):
    """ @param[in] modelMirrorState: MirState object representing an imperfect model
        @param[in] trueMirState: MirState object representing the true mirror
        @param[in] desOrient: a collection of 5 items: pistion, tiltx, tilty, transx, transy. units um and arcsec
    """
    desOrient = desOrient*ConvertOrient
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


class ConvergenceTestBase(MirrorCtrlTestBase):

    def setUp(self):
        self.dispatcher = None
        if type(self) == ConvergenceTestBase:
            # the unit test framework will try to instantiate this class, even though it has no test cases
            # no need to do anything in that case
            return
        self.setVars()
        # first start up the fake galil listening
        galilPort = self.startFakeGalil()
        # connect an actor to it
        d = self.startMirrorCtrl(galilPort = galilPort)
        # after that connection is made, connect a dispatcher to the actor
        d.addCallback(self.startCommander)
        return d

    def _testConv(self, modelMirState, trueMirState, desOrient):
        """ @param[in] modelMirrorState: MirState object representing an imperfect model
            @param[in] trueMirState: MirState object representing the true mirror
            @param[in] desOrient: a collection of 5 items: pistion, tiltx, tilty, transx, transy. units um and arcsec
        """
        desOrient = desOrient*ConvertOrient
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
        self.mirror: mirorCtrl Mirror, e.g. mirrorCtrl.mirrors.mir35mTert.Mirror
        self.mirDev: mirror device, e.g. mirrorCtrl.GalilDevice
        self.name: name of keyword dict for this mirror
        """
        print "MirrorCtrlTestBase.setVars"
        raise NotImplementedError()

    def _testOrient(self, orient):
        """ @param[in] orientation: 5 element orientation in user-friendly units (um and arcsec)
        """
        # set encoder noise to zero
        #self.fakeGalilFactory.proto.noiseRange = 0.#
        print 'testing orient', orient
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
            self.assertTrue(self.dispatcher.model.iter.valueList[0] == nIter)
        d.addCallback(checkResults)   
        self.dispatcher.executeCmd(cmdVar)
        return d

    def _testOrients(self):
        """test a set of orientations
        """
        iterOs = iter(m2TestOrients)
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

    def startFakeGalil(self):
        """Start the fake Galil on a randomly chosen port; return the port number
        """
        print "startFakeGalil()"
        self.fakeGalilFactory = self.fakeGalilFactory(verbose=False, wakeUpHomed=True, mirror=self.trueMirror)
        portObj = reactor.listenTCP(port=0, factory=self.fakeGalilFactory)
        galilPort = portObj.getHost().port
        self.addCleanup(portObj.stopListening)
        print "Started fake Galil on port", galilPort
        return galilPort

    # def testSmallOffset(self):
    #     # test that current local error is immediately applied to small
    #     # moves
    #     pass

class ConvergenceTestActEqEnc(ConvergenceTestBase):
    def setVars(self):
        self.fakeGalilFactory = FakeGalilFactory
        self.trueMirror = mir35mSec.Mirror
        self.mirror = getActEqEncMir(self.trueMirror)
        self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror"    

    def testOrients(self):
        return self._testOrients()    

# class ConvergenceTestRandAct(ConvergenceTestBase):
#     def setVars(self):
#         self.fakeGalilFactory = FakeGalilFactory
#         self.trueMirror = mir35mSec.Mirror
#         self.mirror = getActRandMove(self.trueMirror)
#         self.mirDev = mirrorCtrl.GalilDevice
#         self.name = "mirror"        

#     def testOrients(self):
#         return self._testOrients()

class ConvergenceTestRandAct(ConvergenceTestBase):
    def setVars(self):
        self.fakeGalilFactory = FakeGalilFactory
        self.trueMirror = mir35mSec.Mirror
        self.mirror = getActRandMove(self.trueMirror, seed=45)
        self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror" 

    def testOrients(self):
        return self._testOrients()

class ConvergenceTestPerfect(ConvergenceTestBase):
    """model exactly represents truth, no iterations
    """
    def setVars(self):
        self.fakeGalilFactory = FakeGalilFactory
        self.trueMirror = mir35mSec.Mirror
        self.mirror = copy.deepcopy(mir35mSec.Mirror)
        self.mirDev = mirrorCtrl.GalilDevice
        self.name = "mirror" 

    def testOrients(self):
        return self._testOrients()

    def _testConv(self, modelMirState, trueMirState, desOrient):
        iters = 1
        offset = 'foo'
        return iters, offset


if __name__ == "__main__":
    from unittest import main
    main()