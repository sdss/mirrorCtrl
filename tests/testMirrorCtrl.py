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
    print 'Keyword Reply: ' + msgStr + "\n"



class TestObj(object):
    """Contains a command to be executed, and information on how it should terminate
    """
    def __init__(self, dispatcher, cmdStr, shouldFail=False, modelInfo=None):
        """Inputs:
        cmdStr: the string to send to the actor
        shouldFail: bool, when the command terminates should it have failed?
        modelInfo: a keyword dict of keywords and values expected to be on the model after
            the termination of this command.
        """
        self.deferred = Deferred()
        self.dispatcher = dispatcher
        self.shouldFail = bool(shouldFail)
        self.modelInfo = modelInfo if modelInfo else {}
        self.cmdVar = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
            
    def cmdCB(self, thisCmd):
        """called each time this command changes state
        """
        #print self.dispatcher.model.keyVarDict
        if self.cmdVar.isDone:
            print 'command finished: ', self.cmdVar.cmdStr
            # did the command finish in the expected way?
            result = {'cmdStr':self.cmdVar.cmdStr, 'exitOK': [self.shouldFail, self.cmdVar.didFail]}
            # check the status of the keywords on the model, compare with expected
            for kw, expected in self.modelInfo.iteritems():
                result[kw] = [expected, getattr(self.dispatcher.model, kw).valueList]
            deferred, self.deferred = self.deferred, None
            deferred.callback(result)
        
    def executeCmd(self):
        """execute the command
        """
        print 'running command: ', self.cmdVar.cmdStr
        self.dispatcher.executeCmd(self.cmdVar)
        #return self.deferred
        
        
class TestCollision(object):
    """Receives a list of TestObjs, and will send them in immediate order.
    """
    def __init__(self, testObjList):
        """Inputs:
        testObjList: must be a list of TestObj objects.
        """
        self.testObjList = testObjList
        
    def exectueCmd(self):
        """Start all all commands in order
        """
        deferreds = []
        for obj in testObjList: 
            deferreds.append(obj.executeCmd())
        return gatherResults(deferreds)
        
        
class TestRunner(object):
    """An object to run a series of testObjects or TestCollisions, check the return, then run the next test
    """
    def __init__(self, testList):
        self.deferred = Deferred()
        self.testList = iter(testList)
        self.results = []
        
    def runTests(self, testResult=None):
        """Runs one test at a time
        Inputs:
        testResult =  because this method is run as a callback, it must accept the callback result.
        """
        if testResult:
            self.results.append(testResult)
        try:
            nextCmd = self.testList.next()
        except:
            print 'all tests finished: '
            # all tests are done
            deferred, self.deferred = self.deferred, None
            deferred.callback(self.results) # accumulate results from all tests and pass em on
            return
        nextCmd.deferred.addCallback(self.runTests)
        nextCmd.executeCmd()
        return self.deferred
        
        
class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    #timeout=20

    # 
    Orientation = [10000, 3600, 3600] # 10 mm, 1 deg, 1 deg axes to move
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
        portObj = reactor.listenTCP(port=0, factory=FakeGalilFactory(verbose=False))
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
                
    def xtestIt(self):
        self.mirDev.status.maxIter = 0
        testRunner = TestRunner(
            testList = [
                TestObj(
                    dispatcher = self.dispatcher, 
                    cmdStr = 'move ' + ', '.join([str(x) for x in self.Orientation]),
                    shouldFail=False, 
                    modelInfo={
                        'encMount': self.mirDev.mirror.encoderMountFromOrient(self.mirActor.processOrientation(self.Orientation)),
                        'cmdMount': numpy.around(numpy.asarray(self.mirDev.mirror.actuatorMountFromOrient(self.mirActor.processOrientation(self.Orientation)))/50.)*50.,
                        'iter': 1,
                    }
                ),
            ]
        )
        testRunner.deferred.addCallback(self.doAssertions)
        return testRunner.runTests()
        
    def doAssertions(self, allTestResults):
        """Loop through all results and deem them satisfactory or not
        Inputs:
            allTestResults: a list of dictionaries containing results from each command
            test.  
        """
        print 'model gParST_FSx: ', getattr(self.dispatcher.model, 'gParST_FSx').valueList
        print 'model status: ', getattr(self.dispatcher.model, 'status').valueList
        for cmd in allTestResults:            
            compare = []
            print 'Results from cmd: ', cmd['cmdStr']
            for kw, result in cmd.iteritems():
                if kw=='cmdStr': 
                    continue # skip the command string
                print kw + ': ' + str(result)
                
    def testMove(self):
        self.mirDev.status.maxIter = 0
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
        self.deferred.addCallback(self.checkMoveResults)        
        self.dispatcher.executeCmd(cmdVar)
        return self.deferred

    def checkMoveResults(self, *args):
        self.assertEqual(1, self.dispatcher.model.iter.valueList[0])
        
    def testAgain(self):
        return self.testMove()

    def testAgain1(self):
        return self.testMove()

    def testAgain2(self):
        return self.testMove()
        
    def testAgain3(self):
        return self.testMove()
        

    def cmdCB(self, thisCmd):
        """called each time this command changes state
        """
        #print self.dispatcher.model.keyVarDict
        if thisCmd.isDone:
            d, self.deferred=self.deferred, None
            d.callback('hell yes')             
