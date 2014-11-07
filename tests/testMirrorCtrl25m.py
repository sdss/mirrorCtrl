#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Test the 2.5m mirror controllers, which each have slightly unique behavior
The primary offsets ABC equally in pure piston moves, and has a minimum step size of 50
The secondary is a tip trans mirror and has piezos
"""

import numpy

from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred, gatherResults
from twisted.internet import reactor
from twistedActor import testUtils, log

testUtils.init(__file__)

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from RO.Comm.TwistedTimer import Timer
from opscore.actor import CmdVar

from mirrorCtrl.mirrors import mir25mSec, mir35mTert, mir25mPrim
from mirrorCtrl import MirrorDispatcherWrapper

## speed up fake galil responses
import mirrorCtrl.fakeGalil
mirrorCtrl.fakeGalil.MaxCmdTime = 0.025 # seconds, longest any command may take
# add actuator measurement noise (to kick in piezo moves)
mirrorCtrl.fakeGalil.ADDNOISE = True

def showReply(msgStr, *args, **kwargs): # prints what the dispatcher sees
    print 'Keyword Reply: ' + msgStr

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

class BaseTests(object):
    """Tests a piezo mirror setup
    """
    def setUp(self):
        self.homeStr = 'home A,B,C,D,E'
        self.name = "mirror" #"piezomirror"
        self.dw = MirrorDispatcherWrapper(
            mirror=mir25mSec,
            dictName=self.name
        )
        return self.dw.readyDeferred

    def tearDown(self):
        from twisted.internet import reactor
        delayedCalls = reactor.getDelayedCalls()
        for call in delayedCalls:
            call.cancel()
        return self.dw.close()

    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher

    @property
    def actor(self):
        return self.dw.actorWrapper.actor

    @property
    def fakeGalil(self):
        """Return the fake Galil (instance of FakeGalil)
        """
        return self.dw.actorWrapper.deviceWrapperList[0].controller



class PiezoTests(BaseTests, TestCase):
    def setUp(self):
        self.name = "mirror" #"piezomirror"
        self.dw = MirrorDispatcherWrapper(
            mirror=mir25mSec,
            dictName=self.name
        )
        return self.dw.readyDeferred

    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        log.info("piezoTestHome")
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C,D,E',
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertFalse( 0 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))

        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testSimpleMove(self):
        """Simply test a move and see if it works
        """
        log.info("testSimpleMove")
        d1 = Deferred()
        orientation1 = [-2000.0, 0, 0]
        cmdStr1 = 'move ' + ', '.join([str(x) for x in orientation1])
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = cmdStr1,
                callFunc = CmdCallback(d1),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove1.didFail)
            self.assertEqual(self.dispatcher.model.state.valueList[1], self.dispatcher.model.maxIter.valueList[0])
        d1.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove1)
        return d1

class PrimTests(BaseTests, TestCase):
    def setUp(self):
        self.name = "mirror"
        self.dw = MirrorDispatcherWrapper(
            mirror=mir25mPrim,
            dictName=self.name
        )
        return self.dw.readyDeferred

    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        log.info("piezoTestHome")
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C,D,E,F',
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertFalse( 0 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))

        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testSimpleMove(self):
        """Simply test a move and see if it works
        """
        log.info("testSimpleMove")
        d1 = Deferred()
        orientation1 = [-2000.0, 0, 0]
        cmdStr1 = 'move ' + ', '.join([str(x) for x in orientation1])
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = cmdStr1,
                callFunc = CmdCallback(d1),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove1.didFail)
            self.assertEqual(self.dispatcher.model.state.valueList[1], 1)
            # self.assertFalse(numpy.all([0==err for err in self.dispatcher.model.mountErr.valueList[:]]))
            print "mountErr: ", self.dispatcher.model.mountErr.valueList
            for err in self.dispatcher.model.netMountOffset.valueList[:] + self.dispatcher.model.mountErr.valueList[:]:
                # print "err", err
                self.assertEqual(0,int(err))
            # self.assertFalse(numpy.all([0==err for err in self.dispatcher.model.netMountOffset.valueList[:]]))
            print "netMountOffset: ", self.dispatcher.model.mountErr.valueList
            for mount in self.dispatcher.model.cmdMount.valueList + self.dispatcher.model.actMount.valueList + self.dispatcher.model.encMount.valueList:
                # print "mount", mount
                self.assertEqual(0, round(mount)%50)
        d1.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove1)
        return d1


if __name__ == '__main__':
    from unittest import main
    main()
