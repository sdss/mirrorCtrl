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

def showReply(msgStr, *args, **kwargs):
        print 'reply: ' + msgStr + "\n"

def cb(thisConn): 
    """Generic callback used in TCPConnection, called every time a state changes
    thisConn must have dConn and dDisConn Deferred attributes appended for correct
    execution.
    """
    state, reason = thisConn.fullState
    print "Name %s, Connection state: %s %s\n" % (thisConn._name, state, reason)
    if thisConn.isConnected:
        dConn, thisConn.dConn = thisConn.dConn, None
        dConn.callback(None)
    if state in ('Disconnected', 'Failed'):
        dDisConn, thisConn.dDisConn = thisConn.dDisConn, None
        dDisConn.callback(None)


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
               