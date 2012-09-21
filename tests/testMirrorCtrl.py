"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
import fakeGalil
from twisted.trial import unittest
from twisted.internet.defer import Deferred, gatherResults, maybeDeferred
from twisted.internet.protocol import ProcessProtocol, ServerFactory
import mirrorCtrl
import mirrorCtrl.mirrors.mir25mSec
from twistedActor import ActorDevice
from RO.Comm.TCPConnection import TCPConnection
from opscore.actor import ActorDispatcher, CmdVar

# wanna look? uncomment
# mirrorCtrl.devices.device25mPrim.Mirror.plotMirror()
#...

FakeGalilPort = 8000
FakeGalilHost = 'localhost'
Sec25mUserPort = 2532

def showReply(msgStr, *args, **kwargs): # prints what the dispactcher sees to the screen
    print 'reply: ' + msgStr + "\n"

def connCB(self, *args): # callback for TCP connection state change
    print '%s Device state: %s' % (self._name, self.fullState)
    if self.isConnected:
        print '%s Device Connected on port %s' % (self._name, self.port)
        d, self.connDeferred = self.connDeferred, None
        d.callback('')
    if self.fullState[0] == 'Disconnected':
        d, self.disconnDeferred = self.disconnDeferred, None
        d.callback('')



class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    timeout = 10 # ten seconds before timeout

    def setUp(self):
        # first start up the fake galil listening
        self.setupGalil()
        # connect an actor to it
        d = self.setupActor()
        # after that connection is made, connect a dispatcher to the actor
        d.addCallback(self.setupCommander)
        return d
        
    def tearDown(self):
        # don't exit until all connections are disconnected
        # this may be unnecessary...
        d1, d2 = self.actordisconn, self.cmddisconn
        self.actordisconn, self.cmddisconn = None, None
        return gatherResults([d1, d1])

    def setupGalil(self):
        self.factory = ServerFactory()
        self.factory.protocol = fakeGalil.FakeGalilProtocol
        from twisted.internet import reactor
        self.galilport = reactor.listenTCP(0, self.factory, interface="localhost")
        self.galilportnum = self.galilport.getHost().port
        self.addCleanup(self.galilport.stopListening)
        
    def setupActor(self):
        d = Deferred()
        self.actordisconn = Deferred()
        mirror = mirrorCtrl.mirrors.mir25mSec.Mirror
        self.mirDev = mirrorCtrl.GalilDevice25Sec(
            mirror = mirror,
            host = 'localhost',
            port = self.galilportnum,            
            )
        self.mirDev.conn.connDeferred = d
        self.mirDev.conn.disconnDeferred = self.actordisconn
        self.mirDev.conn._name = 'Actor'
        self.mirDev.conn.addStateCallback(connCB) # will callback upon connection
        self.mirActor = mirrorCtrl.MirrorCtrl(
            device = self.mirDev,
            userPort = Sec25mUserPort        
            )
        self.addCleanup(self.mirDev.conn.disconnect)
        self.addCleanup(self.mirActor.server.close)
        return d
    
    
    def setupCommander(self, *args): # *args for the callback result that we don't care about
        # this doesn't close cleanly
        d = Deferred()
        self.cmddisconn = Deferred()
        self.cmdConn = TCPConnection(
            host = 'localhost',
            port = Sec25mUserPort,
            readLines = True,
            name = "Commander",
        )
        self.dispatcher = ActorDispatcher(
            name = "mirror",
            connection = self.cmdConn,
            logFunc = showReply
        )
        self.cmdConn.connDeferred = d
        self.cmdConn.disconnDeferred = self.cmddisconn
        self.cmdConn.addStateCallback(connCB)
        self.cmdConn.connect()
        self.addCleanup(self.cmdConn.disconnect)
        return d    
                
    def testCmds(self): 
       print 'ready to run tests'
