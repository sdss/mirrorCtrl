#!/usr/bin/env python
"""http://twistedmatrix.com/documents/current/core/howto/trial.html

Tests communication and other behavior between the Actor and Device. Commands are dispatched using a 
dispatcher. 

Note: Orientations tested here are almost certainly unrealistic and these tests were failing.
They were previously passing because we hadn't discovered the problem in mount/phys conversions (mm vs um).  
The tests now work because I unset the actuator/encoder limits of motion. 
"""
import numpy

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred, gatherResults
from twisted.internet import reactor
from opscore.actor import ActorDispatcher, CmdVar
from RO.Comm.TCPConnection import TCPConnection

import mirrorCtrl
from mirrorCtrl.mirrors import mir25mSec, mir35mTert
from mirrorCtrl.fakeGalil import FakeGalil, FakePiezoGalil
from mirrorCtrl.fakeDispatcherWrapper import FakeDispatcherWrapper

def showReply(msgStr, *args, **kwargs): # prints what the dispatcher sees
    print 'Keyword Reply: ' + msgStr

class CmdCallback(object):
    """Call a deferred when a command is finished
    """
    def __init__(self, deferred):
        self.deferred = deferred
    
    def __call__(self, cmd):
        if cmd.isDone:
            deferred, self.deferred = self.deferred, None
            deferred.callback("done")

class GenericTests(TestCase):
    """Tests for each command, and how they behave in collisions
    """
    def setUp(self):
        print "*** setUp"
        self.name = "mirror"
        self.dw = FakeDispatcherWrapper(
            mirror=mir35mTert,
        )
        return self.dw.readyDeferred
    
    def tearDown(self):
        d = self.dw.close()
        print "*** tearDown; d=%s; called=%s" % (d, d.called if d else "?????")
        return d
    
    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher
    
    @property
    def fakeGalil(self):
        """Return the fake Galil (instance of FakeGalil)
        """
        return self.dw.actorWrapper.deviceWrapper.controller

    # def testSingleMove(self):
    #     """Turns iteration off, moves once, 
    #     checks:
    #     1. command finished without failure
    #     2. only one iteration was done
    #     3. the commanded mount on the model matches the expected version, explicitly computed
    #     4. the encoder mount position on the model is within the noise range added by the fakeGalil
    #     """
    #     self.test = 'testSingleMove'
    #     self.dw.actor.dev.galil.status.maxIter = 0 # turn iteration off
    #     d = Deferred()
    #     orientation = [10000, 3600, 3600]
    #     cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
    #     encMount = self.dw.actor.dev.galil.mirror.encoderMountFromOrient(
    #         self.mirActor.processOrientation(orientation)
    #         )
    #     # round to nearest 50 like the 3.5m Tert Galil
    #     cmdMount = numpy.around(numpy.asarray(
    #         self.dw.actor.dev.galil.mirror.actuatorMountFromOrient(self.mirActor.processOrientation(orientation)))/50.
    #         )*50.
        
    #     cmdVar = CmdVar (
    #             actor = self.name,
    #             cmdStr = cmdStr,
    #             callFunc = CmdCallback(d),
    #         )
    #     def checkResults(cb):
    #         """Check results after cmdVar is done
    #         """
    #         self.assertEqual(cmdVar.didFail, False)
    #         self.assertEqual(1, self.dispatcher.model.iter.valueList[0])
    #         self.assertTrue(numpy.array_equal(
    #             cmdMount, self.dispatcher.model.cmdMount.valueList[:]
    #             ))
    #         # encMount should be within the noise range determined
    #         # on the fake Galil
    #         noiseRng = self.fakeGalil.noiseRange # steps
    #         encDiff = numpy.abs(numpy.subtract(encMount, self.dispatcher.model.encMount.valueList[:]))
    #         encTruth = numpy.abs(encDiff) < noiseRng
    #         encInRange = False if False in encTruth else True
    #         self.assertTrue(encInRange)            
    #     d.addCallback(checkResults)        
    #     self.dispatcher.executeCmd(cmdVar)
    #     return d
        
    # def testIterMove(self):
    #     """move with allowed iteration
    #     checks:
    #     1. command finished without failing
    #     2. the iter value on the model is > 1
    #     """
    #     self.test = 'testiterMove'
    #     d = Deferred()
    #     orientation = [10000, 3600, 3600]
    #     cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
    #     cmdVar = CmdVar (
    #             actor = self.name,
    #             cmdStr = cmdStr,
    #             callFunc = CmdCallback(d),
    #         )
    #     def checkResults(cb):
    #         """Check results after cmdVar is done
    #         """
    #         self.assertFalse(cmdVar.didFail)
    #         self.assertTrue(self.dispatcher.model.iter.valueList[0] > 1)
    #     d.addCallback(checkResults)        
    #     self.dispatcher.executeCmd(cmdVar)
    #     return d

    def testActorBypass(self):
        d = Deferred()
        st = "ST"
        galilCmd = "A=1134426; B=1732577; C=867754; XQ #MOVE"
        cmdVar1 = CmdVar (
                actor = self.name,
                cmdStr = 'galil ' + st,
                #callFunc = CmdCallback(d),
            )       
        cmdVar2 = CmdVar (
                actor = self.name,
                cmdStr = 'galil ' + galilCmd,
                #callFunc = CmdCallback(d),
            ) 
        self.dispatcher.executeCmd(cmdVar1)
        reactor.callLater(0.2, self.dispatcher.executeCmd, cmdVar2)
        reactor.callLater(3, d.callback, "go")
        return d

    def testUnHomedMove(self):
        """Set isHomed on the fakeGalil to all False. Try to move.
        checks:
        1. the command fails
        """
        self.test = 'testUnHomedMove'
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        orientation = [10000, 3600, 3600]
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
        self.test = "testMoveTimeout"
        d = Deferred()
        orientation = [10000, 3600, 3600]
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
        self.dw.actor.dev.galil.DevCmdTimeout = 0.01       
        self.dispatcher.executeCmd(cmdVar)
        return d

    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        self.test = 'testHome'
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
        
    def testStatus(self):
        """tests the status command.  Set is homed to false first to verify that most recent values
        are being reported.
        checks:
        1. command completes without failure
        2. isHomed = False for all axes.
        """
        self.test = "testStatus"
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
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
        return d
        
    def testReset(self):
        """Send a reset command.
        checks:
        1. command completes without failure
        2. isHomed = False for all axes
        """
        self.test = "testReset"
        d = Deferred()
        cmdStr = 'reset'        
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
        return d
        
    def testStop(self):
        """Send a stop command.
        checks:
        1. command completes without failure
        """
        self.test="testStop"
        d = Deferred()
        cmdStr = 'reset'        
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
        self.test = "testStopInterrupt"
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation = [10000, 3600, 3600]
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
        self.dispatcher.executeCmd(cmdStop)
        return dBoth        

    def testResetInterrupt(self):
        """Test that a reset command will interrupt a move command. Commands a move then a reset
        immediately afterwards.
        Checks:
        1. the move fails
        2. the reset succeeds.
        3. check that isHomed == False (due to the reset)
        """          
        self.test = "testResetInterrupt"
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = cmdStr,
                callFunc = CmdCallback(d1),
            )
        cmdReset = CmdVar (
                actor = self.name,
                cmdStr = 'reset',
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdMove.didFail)
            self.assertFalse(cmdReset.didFail)     
            self.assertFalse(1 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))
        dBoth.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdMove)
        self.dispatcher.executeCmd(cmdReset)
        return dBoth

    def testCmdQueueHome(self):
        """send a status then a home, 
        home should finish after status.
        """
        self.test = "testCmdQueueHome"
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
        self.dispatcher.executeCmd(cmdHome)
        return dBoth

    def testDoubleQueueMoveMove(self):
        """send status, move, move.
        2nd move should finish, 1st move and status should finish
        """
        self.test = "testDoubleQueueMoveMove"
        d1 = Deferred()
        d2 = Deferred()
        d3 = Deferred()
        dAll = gatherResults([d1,d2, d3])
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = 'move 10001, 3601, 3601',
                callFunc = CmdCallback(d1),
            )
        cmdStatus = CmdVar (
                actor = self.name,
                cmdStr = 'status',
                callFunc = CmdCallback(d2),
            )
        cmdMove2 = CmdVar (
                actor = self.name,
                cmdStr = 'move 10000, 3600, 3600',
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
        self.dispatcher.executeCmd(cmdMove1)
        self.dispatcher.executeCmd(cmdMove2)
        return dAll

    def testDoubleQueueMoveHome(self):
        """send status, move, home.
        move should finish, home should be rejected, and status should finish
        """
        self.test = "testDoubleQueueMoveHome"
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
                cmdStr = 'move 10000, 3600, 3600',
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
        self.dispatcher.executeCmd(cmdMove)
        self.dispatcher.executeCmd(cmdHome)
        return dAll

    def testDoubleQueueHomeMove(self):
        """send status, home, move.
        home should finish, move rejected, and status should finish
        """
        self.test = "testDoubleQueueHomeMove"
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
                cmdStr = 'move 10000, 3600, 3600',
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
        self.dispatcher.executeCmd(cmdHome)
        self.dispatcher.executeCmd(cmdMove)
        return dAll
 
    def testCmdQueueSuperseded(self):
        """send a status then a home then a stop, 
        stop should succeed rest should fail
        """
        self.test = "testCmdQueueSuperseded"
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
        self.dispatcher.executeCmd(cmdHome)
        self.dispatcher.executeCmd(cmdStop)
        return dAll
    
    def testCmdQueueMove(self):
        """send a staus then a move.
        move should finish after status
        """
        self.test = "testCmdQueueMove"
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        cmdMove = CmdVar (
                actor = self.name,
                cmdStr = 'move 10000, 3600, 3600',
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
        self.dispatcher.executeCmd(cmdMove)
        return dBoth
        
    def testStatusCollide(self):
        """Send a status request while a home command is executing.
        Check:
        1. Status succeeds
        2. Home succeeds
        """
        self.test = 'testStatusCollide'
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
        self.dispatcher.executeCmd(cmdStatus)
        return dBoth

    def testMoveSupersede(self):
        """Send two move commands, the second should supersede the first
        """
        self.test = "testMoveSupersede"
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation1 = [10000, 3600, 3600]
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
            self.assertTrue(cmdMove1.didFail)
            self.assertFalse(cmdMove2.didFail)     
        dBoth.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdMove1)
        self.dispatcher.executeCmd(cmdMove2)
        return dBoth        
        
    def testMoveCollide(self):
        """Send a move command while a home command is executing
        Checks:
        1. The move command is cancelled
        2. The home command succeeds
        """
        self.test = "testMoveCollide"
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation = [10000, 3600, 3600]
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
        self.dispatcher.executeCmd(cmdMove)
        return dBoth

    def testBadMove(self):
        """Send a move command with a commanded z rotation (not allowed).
        checks:
        1. command fails
        """
        self.test = "testBadMove"
        # turn off noise added by fakeGalil.  This move should not iterate.
        self.fakeGalil.encRes = self.fakeGalil.encRes*0.
        d = Deferred()
        orientation = [10000, 3600, 3600, 10000, 10000, 3600]
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
        self.test = "testBadHome"
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

class PiezoTests(TestCase):
    """Tests a piezo mirror setup
    """
    def setUp(self):
        self.name = "piezomirror"
        self.dw = FakeDispatcherWrapper(
            mirror=mir25mSec,
            dictName=self.name,
            galilClass=FakePiezoGalil,
        )
        return self.dw.readyDeferred
    
    def tearDown(self):
        self.dw.close()
    
    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher
    
    @property
    def fakeGalil(self):
        """Return the fake Galil (instance of FakeGalil)
        """
        return self.dw.actorWrapper.deviceWrapper.controller

    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        self.test = "piezoTestHome"
        # force all axes on the fakeGalil to unhomed
        self.fakeGalil.isHomed = self.fakeGalil.isHomed*0.
        d = Deferred()
        cmdStr = 'home A,B,C,D,E'        
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
        
    # def testIterMove(self):
    #     """move with allowed iteration
    #     checks:
    #     1. command finished without failing
    #     2. the iter value on the model is > 1
    #     3. the piezo correction is non-zero
    #     """
    #     self.test = "piezoIterMove"
    #     d = Deferred()
    #     orientation = [1000, 360, 360, 1000]
    #     cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
    #     cmdVar = CmdVar (
    #             actor = self.name,
    #             cmdStr = cmdStr,
    #             callFunc = CmdCallback(d),
    #         )
    #     def checkResults(cb):
    #         """Check results after cmdVar is done
    #         """
    #         self.assertFalse(cmdVar.didFail)
    #         self.assertTrue(self.dispatcher.model.iter.valueList[0] > 1)
    #         self.assertTrue(numpy.sum(numpy.abs(self.dispatcher.model.piezoCorr)) > 0)
    #     d.addCallback(checkResults)        
    #     self.dispatcher.executeCmd(cmdVar)
    #     return d

if __name__ == '__main__':
    from unittest import main
    main()
