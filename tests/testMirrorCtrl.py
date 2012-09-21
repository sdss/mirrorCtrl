"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
from twisted.trial import unittest
from twisted.internet.defer import Deferred # , gatherResults, maybeDeferred
from twisted.internet import reactor

import mirrorCtrl
import mirrorCtrl.mirrors.mir25mSec
from mirrorCtrl.fakeGalil import FakeGalilFactory
from RO.Comm.TCPConnection import TCPConnection
from opscore.actor import ActorDispatcher, CmdVar

Sec25mUserPort = 2532

def showReply(msgStr, *args, **kwargs): # prints what the dispactcher sees to the screen
    print 'reply: ' + msgStr + "\n"


class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    timeout = 10 # ten seconds before timeout


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
        mirror = mirrorCtrl.mirrors.mir25mSec.Mirror
        self.mirDev = mirrorCtrl.GalilDevice25Sec(
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
            userPort = Sec25mUserPort        
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
            port = Sec25mUserPort,
            readLines = True,
            name = "Commander",
        )

        self.dispatcher = ActorDispatcher(
            name = "mirror",
            connection = self.cmdConn,
            logFunc = showReply
        )

        d = Deferred()
        def connCallback(conn, d=d):
            print "commander conn state=", conn.state
            if conn.isConnected:
                d.callback("success")

        self.cmdConn.addStateCallback(connCallback)
        self.cmdConn.connect()
        self.addCleanup(self.cmdConn.disconnect)
        return d    
                
    def testCmds(self): 
       print 'ready to run tests'
