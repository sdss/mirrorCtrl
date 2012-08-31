"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""

import sys
import os
import fcntl
import time
import fakeGalil
from twisted.trial import unittest
from twisted.internet.defer import Deferred, maybeDeferred, gatherResults
from twisted.internet import reactor
from twisted.internet.protocol import Factory, Protocol, ClientFactory, ClientCreator, ProcessProtocol, ServerFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet.endpoints import TCP4ClientEndpoint
import subprocess
import mirrorCtrl

from opscore.protocols.parser import ReplyParser,ParseError
#     reply = rParser.parse("tui.operator 911 BossICC : type=decaf;blend = 20:80, Kenyan,Bolivian ; now")
#     for keyword in reply.keywords:
#         print ' ',keyword.name,keyword.values


sys.path.insert(1,'../bin/')
import Prim25m
import Sec25m
import Sec35m
import Tert35m

# wanna look? uncomment
# Prim25m.Mirror.plotMirror()
# Sec25m.Mirror.plotMirror()
# Sec35m.Mirror.plotMirror()
# Tert35m.Mirror.plotMirror()

FakeGalilPort = 8000
FakeGalilHost = 'localhost'
Sec25mUserPort = 2532


class FakeGalilProtocol(ProcessProtocol):
    def outReceived(self, data):
        print 'outReceived:', data
        self.deferred.callback('')
    def errReceived(self, data):
        print 'gotta error:', data    
    def connectionMade(self):
        print 'connection made!'
        
class FakeActorProtocol(ProcessProtocol):
    def outReceived(self, data):
        print 'outReceived:', data
        self.deferred.callback('')
    def errReceived(self, data):
        print 'gotta error:', data    
    def connectionMade(self):
        print 'connection made!'

# set up a protocol and factory to talk to the mirrorCtrl and parse the replies
class Commander(LineReceiver):
    """Sends line to Mirror Controller, parses replies using
    standard opscore parser
    """
    replyParser = ReplyParser()
    
    def connectionMade(self):
        print 'connected!'
            
    def lineReceived(self, line):
        """
        parse reply into keywords and values
        """
        print 'data in!: ', line
        if ':' in line:
            print 'end!'
            #self.deferred.callback('')
    
    def writeSomething(self, cmd, deferred):
        """write a command
        """
        self.deferred = deferred
        self.transport.write(cmd)
    
    
#     def sendLine(self, line):
#         """send a command to the mirror controller
#         """
#         self.transport.write(line)


class CommanderFactory(ClientFactory):
    def buildProtocol(self, addr):
        p = Commander()
        p.factory = self
        return p
    def startedConnecting(self, connector):
        print 'started connecting'
    def clientConnectionFailed(self, connector, reason):
        print 'connection failed %s' % reason
    def clientConnectionLost(self, connector, reason):
        print 'connection lost %s' % reason


def non_block_read(output):
    fd = output.stdout.fileno()
    fl = fcntl.fcntl(fd, fcntl.F_GETFL)
    fcntl.fcntl(fd, fcntl.F_SETFL, fl | os.O_NONBLOCK)
    try:
        return output.stdout.read()
    except:
        return ""

class TestRunner(object):
    def __init__(self, sendList):
        self.sendList = sendList
        self.deferred = Deferred()
        self.deferred.addCallback(self.sendLine)
        self.factory = ChatFactory(self.deferred)
        self.conn = reactor.connectTCP('localhost',
                                  Sec25mUserPort,
                                  self.factory)
    def sendLine(self, humDee):
        """Send a line to the actor
        """
        line = self.sendList[0]
        #self.factory.protocol.sendLine(line)
        #print 'conn', help(self.conn._makeTrasport)
        self.conn.transport.write(line)
        #LineReceiver.sendLine(self.factory.protocol, line)
        #self.factory.protocol.sendLine(line)


class MirrorCtrlTestCase(unittest.TestCase):
    """A series of tests using twisted's trial unittesting.  Simulates
    communication over a network.
    """
    timeout = 30 # ten seconds before timeout
#     def setUp(self):
#         self.deferred = None
#         self.deferred = Deferred()
#         self.deferred.addCallback(self.actorConnected)
#         d1 = Deferred()
#         d1.addCallback(self.connectActor)        
#         #Begin a fakeGalil before each session
#         self.processProtocol = FakeGalilProtocol()
#         self.processProtocol.deferred = d1
#         self.actorProtocol = FakeActorProtocol()
#         self.actorProtocol.deferred = self.deferred
#         cmd = ['python', "/Users/csayres/Desktop/TCC/mirror/tests/fakeGalil.py"]
#         reactor.spawnProcess(self.processProtocol, cmd[0], cmd,
#             env=os.environ
#             )
#         return self.deferred

    def setUp(self):
        self.setUpGalil()
        self.setUpActor()
        #return self.setUpCommander()
        #self.addCleanup(self.cmdConn.disconnect)
        #self.addCleanup(self.actor.server._basicClose())
    
#    def tearDown(self):
    
#         d = defer.maybeDeferred(self.serverPort.stopListening)
#         self.clientConnection.disconnect()
#         return defer.gatherResults([d,
#                                     self.clientDisconnected,
#                                     self.serverDisconnected])
        #self.cmdConn.disconnect()

#        self.actor.server._basicClose()
#        self.serverPort.stopListening()
#        if self.client is not None:

    def setUpGalil(self):
        factory = ServerFactory()
        factory.protocol = fakeGalil.FakeGalilProtocol
        self.serverPort = reactor.listenTCP(0, factory, interface="localhost")
        self.addCleanup(self.serverPort.stopListening)
        print 'Galil starting up on port: %s' % self.serverPort.getHost().port
    
    def setUpActor(self):
        device = mirrorCtrl.GalilDevice25M2(
            mirror = Sec25m.Mirror,
            host = "localhost",
            port = self.serverPort.getHost().port,
        )
        self.actor = mirrorCtrl.MirrorCtrl(
            device = device,
            userPort = Sec25mUserPort
            )
        self.addCleanup(self.actor.server._basicClose)
        print 'Actor starting up on port: %s' % Sec25mUserPort   
            
    def setUpCommander(self):
        #self.cmdFactory = CommanderFactory()
        creator = ClientCreator(reactor, Commander)
        def cb(client):
            print 'client starting'
            self.client = client
            self.addCleanup(self.client.transport.loseConnection)
        return creator.connectTCP('localhost', Sec25mUserPort).addCallback(cb)
        

    def actorConnected(self, foo):
        """
        """
        print 'actor is connected'

    def commanderConnected(self, foo):
        """
        """
        print 'commander is connected'

    def connectActor(self, foo):
        """Connect the actor when ready
        """
        cmd = ['python', "/Users/csayres/Desktop/TCC/mirror/bin/Sec25m.py"]
        reactor.spawnProcess(self.actorProtocol, cmd[0], cmd,
            env=os.environ
            )
            
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
        #for cmd in cmdStrs[0]:
        #self.client.writeSomething(cmdStrs[0], d)
        print 'cmd sent: ', cmdStrs[0]
        #return d
        #self.testRunner = TestRunner(cmdStrs)
        #return self.testRunner.deferred
               