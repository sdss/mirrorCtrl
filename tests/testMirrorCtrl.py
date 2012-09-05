"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
import fakeGalil
from twisted.trial import unittest
from twisted.internet.defer import Deferred, gatherResults
from twisted.internet import reactor
from twisted.internet.protocol import ProcessProtocol, ServerFactory
import mirrorCtrl
import mirrorCtrl.devices.device25mSec
from twistedActor import ActorDevice
from RO.Comm.TCPConnection import TCPConnection
from opscore.actor import ActorDispatcher, CmdVar

# wanna look? uncomment
# mirrorCtrl.devices.device25mPrim.Mirror.plotMirror()
#...

FakeGalilPort = 8000
FakeGalilHost = 'localhost'
Sec25mUserPort = 2532

class MirActor(mirrorCtrl.MirrorCtrl):
    def __init__(self, device, userPort):
        mirrorCtrl.MirrorCtrl.__init__(
            self,
            device = device,
            userPort = userPort       
        )
        def getState(*args, **kwargs):
            print 'actor server state: ', self.server.state
        self.server.stateCallback = getState
    def newUserOutput(self, userID):
        print 'actor server ready?: ', self.server.isReady


class MirDev(mirrorCtrl.GalilDevice25M2):
    def __init__(self, mirror, host, port, isReady):
        mirrorCtrl.GalilDevice25M2.__init__(self,
        mirror = mirror,
        host = host,
        port = port,
        callFunc = self.getState
        )
        self.conn.addStateCallback(self.getState)
        self.d1 = isReady
        
    def getState(self, *args, **kwargs):
        print 'Mirror Device State: ', self.conn.state
        if self.conn.state == 'Connected':
            print 'mirror device connected to Galil'
            self.d1.callback('')
            


class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    timeout = 10 # ten seconds before timeout

    def setUp(self):
        self.setupGalil()
        galilConn = Deferred()
        self.mirConn = Deferred()
        self.mirConn.addCallbacks(self.success, self.fail)
        galilConn.addCallbacks(self.setupCommander, self.fail)
        self.setupActor(galilConn)
        return self.mirConn
    
    def tearDown(self):
        self.mirConn = None

    
    def success(self, input = None):
        print
        print 'Callback Success, states:'
        print 'Fake Galil Server Host: ', self.serverPort.getHost()
        print 'Mirror Device Connection?: ', self.device.conn.isConnected
        print 'Actor Server state: ', self.actor.server.fullState
        print 'Dispatcher Connection?: ', self.connection.isConnected       
        print 'starting tests'
        
    
    def fail(self, err):
        print
        print 'Callback Failure'
        print err
        

    def setupGalil(self):
        print
        print 'setting up Galil'
        factory = ServerFactory()
        factory.protocol = fakeGalil.FakeGalilProtocol
        def connLost(self, reason):
            print 'Galil Connection Lost: ', reason
        def connectionMade(self, reason):
            print 'Galil Connection Made'
        factory.protocol.connectionLost = connLost
        factory.protocol.connectionMade = connectionMade
        self.serverPort = reactor.listenTCP(0, factory, interface="localhost")
        self.addCleanup(self.serverPort.stopListening)
        print 'Galil starting up on port: %s' % self.serverPort.getHost().port
    
    def setupActor(self, d1):
        print
        print 'setting up actor'
        self.device = MirDev(
            mirror = mirrorCtrl.devices.device25mSec.Mirror,
            host = "localhost",
            port = self.serverPort.getHost().port,
            isReady = d1
        )
        self.actor = MirActor(
            device = self.device,
            userPort = Sec25mUserPort
        )      
        self.addCleanup(self.actor.server._basicClose)
        print 'Actor starting up on port: %s' % Sec25mUserPort
            
    def setupCommander(self, *args, **kwargs):
        print 'setting up Commander'
        self.connection = TCPConnection(
            host = 'localhost',
            port = Sec25mUserPort,
            stateCallback = self.connStateCallback,
            readLines = True,
        )
        
        self.dispatcher = ActorDispatcher(
            name = "mirror",
            connection = self.connection,
            logFunc = self.showReply,
        )
        
        self.connection.connect()
        self.addCleanup(self.connection.disconnect())
        
    def showReply(self, msgStr, *args, **kwargs):
        print 'reply: ' + msgStr + "\n"

    def connStateCallback(self, conn):
        state, reason = conn.fullState
        print "Dispatcher Connection state: %s %s\n" % (state, reason)
        if state == 'Connected':
            self.mirConn.callback('')
            
                
    def testCmds(self):
        """Test a series of commands, wait for successful completion
        """
        #d = Deferred()
        cmdStrs = [
            "move 4,4,4",
            "home A,B,C",
            "status",
            "showparams",
            "stop"
            ]
        print 'cmd sent: ', cmdStrs[0]
        cmdVar = CmdVar (
            actor = "mirror",
            cmdStr = cmdStrs[0],
        )
        self.dispatcher.executeCmd(cmdVar)
        #self.actorDevice.startCmd(cmdStrs[0])
        return Deferred()
               