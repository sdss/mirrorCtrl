"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
import fakeGalil
from twisted.trial import unittest
from twisted.internet.defer import Deferred, gatherResults, maybeDeferred
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

def serverConnLost(self, *args):
    print 'Galil server Connection lost!: '
    d, self.factory.onConnectionLost = self.factory.onConnectionLost, None
    d.callback(self)

def serverConnMade(self, *args):
    print 'Galil server Connection made!'
    d, self.factory.onConnectionMade = self.factory.onConnectionMade, None
    d.callback(self)


class MirDev(mirrorCtrl.GalilDevice25Sec):
    def __init__(self, mirror, host, port, isReady):
        mirrorCtrl.GalilDevice25Sec.__init__(self,
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
        # first start up the fake galil listening
        d1 = maybeDeferred(self.setupGalil)
        d2 = maybeDeferred(self.setupActor)
        d3 = maybeDeferred(self.setupCommander)
        return gatherResults([d1, d2, d3])
        
    def tearDown(self):
        d1 = self.teardownGalil()
        d2 = self.teardownActor()
        d3 = self.teardownCommander()
        return gatherResults([d1, d2, d3])
        
    def setupGalil(self):
        self.factory = ServerFactory()
        self.factory.onConnectionLost = Deferred()
        self.factory.onConnectionMade = Deferred()
        self.factory.protocol = fakeGalil.FakeGalilProtocol
        self.factory.protocol.connectionLost = serverConnLost
        self.factory.protocol.connectionMade = serverConnMade
        from twisted.internet import reactor
        self.galilport = reactor.listenTCP(0, self.factory, interface="localhost")
        self.galilportnum = self.galilport.getHost().port
        return self.factory.onConnectionMade
    
    def teardownGalil(self):
        galilport, self.galilport = self.galilport, None
        d = maybeDeferred(galilport.stopListening)
        return gatherResults([d, self.factory.onConnectionLost])
        
    def setupActor(self):
        dConn = Deferred()
        dDisConn = Deferred()
        mirror = mirrorCtrl.devices.device25mSec.Mirror
        self.mirDev = mirrorCtrl.GalilDevice25M2(
            mirror = mirror,
            host = 'localhost',
            port = self.galilportnum,
            callFunc = None,            
            )
        self.mirDev.conn._name = 'mirror ctrl'
        self.mirDev.conn.addStateCallback(cb)
        self.mirDev.conn.dConn = dConn
        self.mirDev.conn.dDisConn = dDisConn
        self.mirActor = mirrorCtrl.MirrorCtrl(
            device = self.mirDev,
            userPort = Sec25mUserPort        
            )
        return self.mirDev.conn.dConn
    
    def teardownActor(self):
        self.mirDev.conn.disconnect()
        d = maybeDeferred(self.mirActor.server.close)
        return gatherResults([d, self.mirDev.conn.dDisConn])
    
    def setupCommander(self):
        dConn = Deferred()
        dDisConn = Deferred()    
        self.cmdConn = TCPConnection(
            host = 'localhost',
            port = Sec25mUserPort,
            readLines = True,
            name = "commander",
        )
        self.cmdConn.addStateCallback(cb) # state cb receives self
        # append a deferred to keep track of
        self.cmdConn.dConn = dConn
        self.cmdConn.dDisConn = dDisConn
        self.dispatcher = ActorDispatcher(
            name = "mirror",
            connection = self.cmdConn,
            logFunc = showReply
        )
        
        self.cmdConn.connect()
        return dConn
    
    def teardownCommander(self):
        self.cmdConn.disconnect()
        return self.cmdConn.dDisConn
                
    def testCmds(self):
        print 'mir - device conn status', self.mirActor.showDevConnStatus()
        print 'dev conn', self.mirDev.conn.fullState
        print 'cmd conn status', self.cmdConn.fullState
        cmdVar = CmdVar (
            actor = "mirror",
            cmdStr = 'status',
        )
        self.dispatcher.executeCmd(cmdVar)
        return Deferred()
               
