#!/usr/bin/env python2
from __future__ import division, absolute_import
"""http://twistedmatrix.com/documents/current/core/howto/trial.html

Tests communication and other behavior between the Actor and Device. Commands are dispatched using a
dispatcher.
"""

from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred, gatherResults
from twisted.internet import reactor
from twistedActor import testUtils, log

testUtils.init(__file__)

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from RO.Comm.TwistedTimer import Timer
from opscore.actor import CmdVar

from mirrorCtrl.mirrors import mir35mTert, mir25mSec, mir25mPrim
from mirrorCtrl import MirrorDispatcherWrapper

## speed up fake galil responses
import mirrorCtrl.fakeGalil
mirrorCtrl.fakeGalil.MaxCmdTime = 0.025 # seconds, longest any command may take
mirrorCtrl.fakeGalil.ADDNOISE = True # fake a noisy encoder reading
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

class GenericTests(TestCase):
    """Tests for each command, and how they behave in collisions
    """
    def setUp(self):
        self.name = "mirror"
        self.dw = MirrorDispatcherWrapper(
            mirror=mir35mTert,
        )
        return self.dw.readyDeferred

    def tearDown(self):
        return self.dw.close()

    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher

    @property
    def fakeGalil(self):
        """Return the fake Galil (instance of FakeGalil)
        """
        return self.dw.actorWrapper.deviceWrapperList[0].controller

    @property
    def actor(self):
        return self.dw.actorWrapper.actor

    def testActorBypass(self):
        log.info("testActorBypass")
        d = Deferred()
        d2 = Deferred()
        devName = self.actor.dev.nameDict.keys()[0] #just one device
        st = "ST"
        galilCmd = "A=1134426; B=1732577; C=867754; XQ #MOVE"
        cmdVar1 = CmdVar (
                actor = self.name,
                cmdStr =  devName + ' ' + st,
                callFunc = CmdCallback(d2),
            )
        cmdVar2 = CmdVar (
                actor = self.name,
                cmdStr = devName + ' ' + galilCmd,
                callFunc = CmdCallback(d),
            )
        self.dispatcher.executeCmd(cmdVar1)
        reactor.callLater(0.2, self.dispatcher.executeCmd, cmdVar2)
        return gatherResults([d, d2])

    def testUnHomedMove(self):
        """Set isHomed on the fakeGalil to all False. Try to move.
        checks:
        1. the command fails
        """
        log.info('testUnHomedMove')
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        orientation = [-2000.0, 150.0, 860.0]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)
        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testMoveTimeout(self):
        """Let a move command time out, be sure it is handled correctly
        """
        log.info("testMoveTimeout")
        d = Deferred()
        orientation = [-2000.0, 150.0, 860.0]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)
        d.addCallback(checkResults)
        # set timeout to a very small number
        self.dw.actor.galil.DevCmdTimeout = 0.3
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        log.info('testHome')
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdStr = 'home A,B,C'
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
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

    def testBadHomeNoArgs(self):
        """Sets isHomed to false then tests home command with no actuators specified.
        checks:
        1. command fails
        2. all axes are set to homed on the model.
        """
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdStr = 'home'
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)

        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testStatus(self):
        """tests the status command.  Set is homed to false first to verify that most recent values
        are being reported.
        checks:
        1. command completes without failure
        2. isHomed = False for all axes.
        """
        d = Deferred()
        log.info("testStatus")
        # force all axes on the fakeGalil to unhomed
        def runWhenReady(foo=None):
            # force all axes on the fakeGalil to unhomed
            self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
            cmdStr = 'status'
            cmdVar = CmdVar (
                    actor = self.name,
                    cmdStr = cmdStr,
                    callFunc = CmdCallback(d),
                )
            def checkResults(cb):
                """Check results after cmdVar is done
                """
                self.assertFalse(cmdVar.didFail)
                self.assertFalse( 1 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))

            d.addCallback(checkResults)
            self.dispatcher.executeCmd(cmdVar)
        Timer(1., runWhenReady) # give it a second for initialization to succeed.
        return d


    def testStop(self):
        """Send a stop command.
        checks:
        1. command completes without failure
        """
        self.test="testStop"
        d = Deferred()
        cmdStr = 'stop'
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)

        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testStopInterrupt(self):
        """Test that a stop command will interrupt a move command. Commands a move then a stop
        immediately afterwards.
        Checks:
        1. the move fails
        2. the stop succeeds.
        """
        log.info("testStopInterrupt")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation = [-2000.0, 150.0, 860.0]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d1),
            )
        cmdStop = CmdVar (
                actor = self.name,
                cmdStr = 'stop',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdMove.didFail)
            self.assertFalse(cmdStop.didFail)
        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove)
        Timer(0.02, self.dispatcher.executeCmd, cmdStop)
        return dBoth


    def testCmdQueueHome(self):
        """send a status then a home,
        home should finish after status.
        """
        log.info("testCmdQueueHome")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        cmdHome = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdHome.didFail)
            self.assertFalse(cmdStatus.didFail)

        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdStatus)
        Timer(0.02, self.dispatcher.executeCmd, cmdHome)
        # self.dispatcher.executeCmd(cmdHome)
        return dBoth

    def testDoubleQueueMoveMove(self):
        """send status, move, move.
        2nd move should finish, 1st move and status should finish
        """
        log.info("testDoubleQueueMoveMove")
        d1 = Deferred()
        d2 = Deferred()
        d3 = Deferred()
        dAll = gatherResults([d1,d2, d3])
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = 'move 1001, 601, 601',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        cmdMove2 = CmdVar (
                actor = self.name,
                cmdStr = 'move -2000, 150, 860',
                callFunc = CmdCallback(d3),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdMove1.didFail)
            self.assertFalse(cmdStatus.didFail)
            self.assertFalse(cmdMove2.didFail)

        dAll.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdStatus)
        # self.dispatcher.executeCmd(cmdMove1)
        Timer(0.02, self.dispatcher.executeCmd, cmdMove1)
        Timer(0.04, self.dispatcher.executeCmd, cmdMove2)
        # self.dispatcher.executeCmd(cmdMove2)
        return dAll

    def testDoubleQueueMoveHome(self):
        """send status, move, home.
        move should finish, home should be rejected, and status should finish
        """
        log.info("testDoubleQueueMoveHome")
        d1 = Deferred()
        d2 = Deferred()
        d3 = Deferred()
        dAll = gatherResults([d1,d2, d3])
        cmdHome = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = 'move -2000, 150, 860',
                callFunc = CmdCallback(d3),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove.didFail)
            self.assertFalse(cmdStatus.didFail)
            self.assertTrue(cmdHome.didFail)

        dAll.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdStatus)
        # self.dispatcher.executeCmd(cmdMove)
        # self.dispatcher.executeCmd(cmdHome)
        Timer(0.02, self.dispatcher.executeCmd, cmdMove)
        Timer(0.04, self.dispatcher.executeCmd, cmdHome)
        return dAll

    def testDoubleQueueHomeMove(self):
        """send status, home, move.
        home should finish, move rejected, and status should finish
        """
        log.info("testDoubleQueueHomeMove")
        d1 = Deferred()
        d2 = Deferred()
        d3 = Deferred()
        dAll = gatherResults([d1,d2, d3])
        cmdHome = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = 'move -2000, 150, 860',
                callFunc = CmdCallback(d3),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdHome.didFail)
            self.assertFalse(cmdStatus.didFail)
            self.assertTrue(cmdMove.didFail)

        dAll.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdStatus)
        # self.dispatcher.executeCmd(cmdHome)
        # self.dispatcher.executeCmd(cmdMove)
        Timer(0.02, self.dispatcher.executeCmd, cmdHome)
        Timer(0.04, self.dispatcher.executeCmd, cmdMove)
        return dAll

    def testCmdQueueSuperseded(self):
        """send a status then a home then a stop,
        stop should succeed rest should fail
        """
        log.info("testCmdQueueSuperseded")
        d1 = Deferred()
        d2 = Deferred()
        d3 = Deferred()
        dAll = gatherResults([d1,d2, d3])
        def runWhenReady():
            cmdHome = CmdVar (
                    actor = self.name,
                    cmdStr = 'home A,B,C',
                    callFunc = CmdCallback(d1),
                )
            cmdStatus = CmdVar (
                    actor = self.name,
                    cmdStr = 'status',
                    callFunc = CmdCallback(d2),
                )
            cmdStop = CmdVar (
                    actor = self.name,
                    cmdStr = 'stop',
                    callFunc = CmdCallback(d3),
                )
            def checkResults(cb):
                """Check results after cmdVar is done
                """
                self.assertTrue(cmdHome.didFail)
                self.assertTrue(cmdStatus.didFail)
                self.assertFalse(cmdStop.didFail)

            dAll.addCallback(checkResults)
            self.dispatcher.executeCmd(cmdStatus)
            # Timer(0.02, self.dispatcher.executeCmd, cmdHome)
            # Timer(0.04, self.dispatcher.executeCmd, cmdStop)
            self.dispatcher.executeCmd(cmdHome)
            self.dispatcher.executeCmd(cmdStop)
        Timer(1., runWhenReady) # give it a second for initialization to succeed.
        return dAll

    def testCmdQueueMove(self):
        """send a staus then a move.
        move should finish after status
        """
        log.info("testCmdQueueMove")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = 'move -2000, 150, 860',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove.didFail)
            self.assertFalse(cmdStatus.didFail)

        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdStatus)
        # self.dispatcher.executeCmd(cmdMove)
        Timer(0.04, self.dispatcher.executeCmd, cmdMove)
        return dBoth

    def testStatusCollide(self):
        """Send a status request while a home command is executing.
        Check:
        1. Status succeeds
        2. Home succeeds
        """
        log.info('testStatusCollide')
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        cmdHome = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdHome.didFail)
            self.assertFalse(cmdStatus.didFail)

        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdHome)
        Timer(0.02, self.dispatcher.executeCmd, cmdStatus)
        # self.dispatcher.executeCmd(cmdStatus)
        return dBoth

    def testSimpleMove(self):
        """Simply test a move and see if it works
        """
        log.info("testSimpleMove")
        d1 = Deferred()
        orientation1 = [-2000.0, 150.0, 860.0]
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
        d1.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove1)
        return d1

    def testMoveSupersede(self):
        """Send two move commands, the second should supersede the first
        """
        log.info("testMoveSupersede")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation1 = [-2000.0, 150.0, 860.0]
        orientation2 = [num-50 for num in orientation1]
        cmdStr1 = 'move ' + ', '.join([str(x) for x in orientation1])
        cmdStr2 = 'move ' + ', '.join([str(x) for x in orientation2])
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = cmdStr1,
                callFunc = CmdCallback(d1),
            )
        cmdMove2 = CmdVar (
                actor = self.name,
                cmdStr = cmdStr2,
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove1.didFail)
            self.assertFalse(cmdMove2.didFail)
        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove1)
        Timer(0.02, self.dispatcher.executeCmd, cmdMove2)
        # self.dispatcher.executeCmd(cmdMove2)
        return dBoth

    def testMoveCollide(self):
        """Send a move command while a home command is executing
        Checks:
        1. The move command is cancelled
        2. The home command succeeds
        """
        log.info("testMoveCollide")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation = [-2000.0, 150.0, 860.0]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d1),
            )
        cmdHome = CmdVar (
                actor = self.name,
                cmdStr = 'home A,B,C',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdMove.didFail)
            self.assertFalse(cmdHome.didFail)
        dBoth.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdHome)
        Timer(0.04, self.dispatcher.executeCmd, cmdMove)
        # self.dispatcher.executeCmd(cmdMove)
        return dBoth

    def testBadMove(self):
        """Send a move command with a commanded z rotation (not allowed).
        checks:
        1. command fails
        """
        log.info("testBadMove")
        self.fakeGalil.encRes = self.fakeGalil.encRes*0.
        d = Deferred()
        orientation = [-2000.0, 150.0, 860.0, 0.0, -2000.0, 48.0]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)
        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testBadHome(self):
        """Try to home non-existant axis D.
        checks:
        1. command fails
        """
        log.info("testBadHome")
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdStr = 'home A,B,C,D'
        cmdVar = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)

        d.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return d

class Base25mTests(object):
    """Tests a for 2.5m mirrors
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



class PiezoTests(Base25mTests, TestCase):
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

class PrimTests(Base25mTests, TestCase):
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
            # print "mountErr: ", self.dispatcher.model.mountErr.valueList
            for err in self.dispatcher.model.netMountOffset.valueList[:] + self.dispatcher.model.mountErr.valueList[:]:
                # print "err", err
                self.assertEqual(0,int(err))
            # self.assertFalse(numpy.all([0==err for err in self.dispatcher.model.netMountOffset.valueList[:]]))
            # print "netMountOffset: ", self.dispatcher.model.mountErr.valueList
            for mount in self.dispatcher.model.cmdMount.valueList + self.dispatcher.model.actMount.valueList + self.dispatcher.model.encMount.valueList:
                # print "mount", mount
                self.assertEqual(0, round(mount)%50)
        d1.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdMove1)
        return d1


if __name__ == '__main__':
    from unittest import main
    main()
