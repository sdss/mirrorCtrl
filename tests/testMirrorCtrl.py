"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
from twisted.trial import unittest
from twisted.internet.defer import Deferred, gatherResults, maybeDeferred
from twisted.internet import reactor
import numpy

import mirrorCtrl
import mirrorCtrl.mirrors.mir35mTert
from mirrorCtrl.fakeGalil import FakeGalilFactory
from RO.Comm.TCPConnection import TCPConnection
from opscore.actor import ActorDispatcher, CmdVar, AllCodes

Tert35mUserPort = 2532

def showReply(msgStr, *args, **kwargs): # prints what the dispactcher sees to the screen
    return
    print 'Keyword Reply: ' + msgStr + "\n"
    

        
class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    def setUp(self):
        print "setUp()"
        self.dispatcher = None
        # first start up the fake galil listening
        galilPort = self.startFakeGalil()
        # connect an actor to it
        d = self.startMirrorCtrl(galilPort = galilPort)
        # after that connection is made, connect a dispatcher to the actor
        d.addCallback(self.startCommander)
        return d
        
    def tearDown(self):
        """Tear down things; most of the work is done by addCleanup
        """
        print "tearDown()"
        if self.dispatcher is not None:
            self.dispatcher.disconnect()

    def startFakeGalil(self):
        """Start the fake Galil on a randomly chosen port; return the port number
        """
        print "startFakeGalil()"
        self.fakeGalilFactory = FakeGalilFactory(verbose=False, wakeUpHomed=True)
        portObj = reactor.listenTCP(port=0, factory=self.fakeGalilFactory)
        galilPort = portObj.getHost().port
        self.addCleanup(portObj.stopListening)
        print "Started fake Galil on port", galilPort
        return galilPort
        
    def startMirrorCtrl(self, galilPort):
        """Start mirror controller
        """
        print "startMirrorCtrl(galilPort=%r)" % (galilPort,)
        mirror = mirrorCtrl.mirrors.mir35mTert.Mirror
        self.mirDev = mirrorCtrl.GalilDevice35Tert(
            mirror = mirror,
            host = 'localhost',
            port = galilPort,            
        )
        
        d = Deferred()
        def connCallback(conn, d=d):
            print "mirror controller conn state=", conn.state
            if conn.isConnected:
                d.callback("success")
        
        self.mirDev.conn.addStateCallback(connCallback)
        self.mirActor = mirrorCtrl.MirrorCtrl(
            device = self.mirDev,
            userPort = Tert35mUserPort        
            )
        self.addCleanup(self.mirDev.conn.disconnect)
        self.addCleanup(self.mirActor.server.close)
        return d

    def startCommander(self, *args): # *args for the callback result that we don't care about
        """Start a commander"""
        print "startCommander(*args=%r)" % (args,)
        # this doesn't close cleanly
        self.cmdConn = TCPConnection(
            host = 'localhost',
            port = Tert35mUserPort,
            readLines = True,
            name = "Commander",
        )

        self.dispatcher = ActorDispatcher(
            name = "mirror",
            connection = self.cmdConn,
            logFunc = showReply
        )
        d = Deferred()
        def allReady(cb):
            # so fire the callback
            d.callback('whooohooo')
        def getStatus(conn):
            print "commander conn state=", conn.state
            if conn.isConnected:
                print 'Querying Device for status'
                cmdVar = CmdVar (
                    actor = "mirror",
                    cmdStr = 'status',
                    callFunc = getParams,
                )
                self.dispatcher.executeCmd(cmdVar)
        def getParams(cb):
            print 'Querying Device for params'
            cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = 'showparams',
                callFunc = allReady,
            )
            self.dispatcher.executeCmd(cmdVar)
        self.cmdConn.addStateCallback(getStatus)
        self.cmdConn.connect()
        self.addCleanup(self.cmdConn.disconnect)
        return d    


    def cmdCB(self, thisCmd):
        """called each time cmd changes state
        """
        #print self.dispatcher.model.keyVarDict
        if thisCmd.isDone:
            d, self.deferred=self.deferred, None
            d.callback('hell yes')                  
                
    def testSingleMove(self):
        """Turns iteration off, moves once, 
        checks:
        1. command finished without failure
        2. only one iteration was done
        3. the commanded mount on the model matches the expected version, explicitly computed
        4. the encoder mount position on the model is within the noise range added by the fakeGalil
        """
        self.mirDev.status.maxIter = 0 # turn iteration off
        self.deferred = Deferred()
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])
        encMount = self.mirDev.mirror.encoderMountFromOrient(
            self.mirActor.processOrientation(orientation)
            )
        # round to nearest 50 like the 3.5m Tert Galil
        cmdMount = numpy.around(numpy.asarray(
            self.mirDev.mirror.actuatorMountFromOrient(self.mirActor.processOrientation(orientation)))/50.
            )*50.
        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertEqual(cmdVar.didFail, False)
            self.assertEqual(1, self.dispatcher.model.iter.valueList[0])
            self.assertTrue(numpy.array_equal(
                cmdMount, self.dispatcher.model.cmdMount.valueList[:]
                ))
            # encMount should be within the noise range determined
            # on the fake Galil
            noiseRng = self.fakeGalilFactory.proto.noiseRange # steps
            encDiff = numpy.abs(numpy.subtract(encMount, self.dispatcher.model.encMount.valueList[:]))
            encTruth = numpy.abs(encDiff) < noiseRng
            encInRange = False if False in encTruth else True
            self.assertTrue(encInRange)            
        self.deferred.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testIterMove(self):
        """move with allowed iteration
        checks:
        1. command finished without failing
        2. the iter value on the model is > 1
        """
        self.deferred = Deferred()
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertTrue(self.dispatcher.model.iter.valueList[0] > 1)
        self.deferred.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred

    def testCleanMove(self):
        """Turn off noise added by fakeGalil so the first move goes to
        exactly the right spot. This should cause a move to finish without needing
        to iterate
        checks:
        1. command finishes without failure
        2. iter on the model = 1
        3. the maxIter on the device is > 1
        """
        # turn off noise added by fakeGalil.  This move should not iterate.
        self.fakeGalilFactory.proto.encRes = self.fakeGalilFactory.proto.encRes*0.
        self.deferred = Deferred()
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertTrue(self.dispatcher.model.iter.valueList[0], 1)
            self.assertTrue(self.dispatcher.model.iter.valueList[0] < self.mirDev.status.maxIter)
        self.deferred.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred

    def testUnHomedMove(self):
        """Set isHomed on the fakeGalil to all False. Try to move.
        checks:
        1. the command fails
        """
        # force all axes on the fakeGalil to unhomed
        self.fakeGalilFactory.proto.isHomed = self.fakeGalilFactory.proto.isHomed*0.
        self.deferred = Deferred()
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertTrue(cmdVar.didFail)
        self.deferred.addCallback(checkResults)        
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testHome(self):
        """Sets isHomed to false then tests home command.
        checks:
        1. command doesn't fail
        2. all axes are set to homed on the model.
        """
        # force all axes on the fakeGalil to unhomed
        self.fakeGalilFactory.proto.isHomed = self.fakeGalilFactory.proto.isHomed*0.
        self.deferred = Deferred()
        cmdStr = 'home A,B,C'        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertFalse( 0 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))
            
        self.deferred.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testStatus(self):
        """tests the status command.  Set is homed to false first to verify that most recent values
        are being reported.
        checks:
        1. command completes without failure
        2. isHomed = False for all axes.
        """
        # force all axes on the fakeGalil to unhomed
        self.fakeGalilFactory.proto.isHomed = self.fakeGalilFactory.proto.isHomed*0.
        self.deferred = Deferred()
        cmdStr = 'status'        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertFalse( 1 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))
            
        self.deferred.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testReset(self):
        """Send a reset command.
        checks:
        1. command completes without failure
        2. isHomed = False for all axes
        """
        self.deferred = Deferred()
        cmdStr = 'reset'        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            self.assertFalse( 1 in self.dispatcher.model.axisHomed.valueList[:], msg=str(self.dispatcher.model.axisHomed.valueList[:]))
            
        self.deferred.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testStop(self):
        """Send a stop command.
        checks:
        1. command completes without failure
        """
        self.deferred = Deferred()
        cmdStr = 'reset'        
        cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdVar.didFail)
            
        self.deferred.addCallback(checkResults)
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred
        
    def testStopInterrupt(self):
        """Test that a stop command will interrupt a move command. Commands a move then a stop
        immediately afterwards.
        Checks:
        1. the move fails
        2. the stop succeeds.
        """          
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        def cmdCB1(thisCmd):
            """callback associated with first cmd
            """
            if thisCmd.isDone:
                d1.callback('hell yes') 
        def cmdCB2(thisCmd):
            """callback associated with second cmd
            """
            if thisCmd.isDone:
                d2.callback('hell yes') 
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdMove = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = cmdCB1,
            )
        cmdStop = CmdVar (
                actor = "mirror",
                cmdStr = 'stop',
                callFunc = cmdCB2,
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
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        def cmdCB1(thisCmd):
            """callback associated with first cmd
            """
            if thisCmd.isDone:
                d1.callback('hell yes') 
        def cmdCB2(thisCmd):
            """callback associated with second cmd
            """
            if thisCmd.isDone:
                d2.callback('hell yes') 
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdMove = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = cmdCB1,
            )
        cmdReset = CmdVar (
                actor = "mirror",
                cmdStr = 'reset',
                callFunc = cmdCB2,
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
        
    def testStatusCollide(self):
        """Send a status request while a home command is executing.
        Check:
        1. Status succeeds
        2. Home succeeds
        """
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        def cmdCB1(thisCmd):
            """callback associated with first cmd
            """
            if thisCmd.isDone:
                d1.callback('hell yes') 
        def cmdCB2(thisCmd):
            """callback associated with second cmd
            """
            if thisCmd.isDone:
                d2.callback('hell yes')    
        cmdHome = CmdVar (
                actor = "mirror",
                cmdStr = 'home A,B,C',
                callFunc = cmdCB1,
            )
        cmdStatus = CmdVar (
                actor = "mirror",
                cmdStr = 'status',
                callFunc = cmdCB2,
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
        
    def testMoveCollide(self):
        """Send a move command while a home command is executing
        Checks:
        1. The move command is cancelled
        2. The home command succeeds
        """
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        def cmdCB1(thisCmd):
            """callback associated with first cmd
            """
            if thisCmd.isDone:
                d1.callback('hell yes') 
        def cmdCB2(thisCmd):
            """callback associated with second cmd
            """
            if thisCmd.isDone:
                d2.callback('hell yes') 
        orientation = [10000, 3600, 3600]
        cmdStr = 'move ' + ', '.join([str(x) for x in orientation])        
        cmdMove = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = cmdCB1,
            )
        cmdHome = CmdVar (
                actor = "mirror",
                cmdStr = 'home A,B,C',
                callFunc = cmdCB2,
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