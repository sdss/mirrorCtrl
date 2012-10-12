"""http://twistedmatrix.com/documents/current/core/howto/trial.html
"""
from twisted.trial import unittest
from twisted.internet.defer import Deferred # , gatherResults, maybeDeferred
from twisted.internet import reactor

import mirrorCtrl
import mirrorCtrl.mirrors.mir35mTert
from mirrorCtrl.fakeGalil import FakeGalilFactory
from RO.Comm.TCPConnection import TCPConnection
from opscore.actor import ActorDispatcher, CmdVar, AllCodes

Tert35mUserPort = 2532

def showReply(msgStr, *args, **kwargs): # prints what the dispactcher sees to the screen
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
        def connCallback(conn, d=d):
            print "commander conn state=", conn.state
            if conn.isConnected:
                d.callback("success")

        self.cmdConn.addStateCallback(connCallback)
        self.cmdConn.connect()
        self.addCleanup(self.cmdConn.disconnect)
        return d    
                
    def testDispactched(self): 
        print 'ready to run tests'
        self.d = Deferred()
        self.runIndivCmds(self.d)


    def runIndivCmds(self, d):
        """Dispatch a set of commands one at a time, wait for completion before
        sending the next
        
        Input:
        d = a deferred to keep track of
        """
        self.cmdVars = []
        cmdStrs = [
            'move 1,2,3',
            'home A,B,C',
            'status',
            'showparams',
            'stop',
            'reset'
        ]
        for cmdStr in cmdStrs:
            cmd = CmdVar (
                actor = "mirror",
                cmdStr = cmdStr,
                callFunc = self.cmdCB,
            )
            self.cmdVars.append(cmd)
        self.cmdIter = iter(self.cmdVars)
        d.addBoth(self.allPass)
        d.addBoth(self.assertEqual, True) # true is expected from allPass
        firstCmd = self.cmdIter.next()
        print 'Sending Cmd: ', firstCmd.cmdStr
        self.dispatcher.executeCmd(firstCmd)


    def allPass(self, *args):
        """Return True if all commands on the stack passed
        """
        anyFail = [cmd.didFail for cmd in self.cmdVars]
        if True in anyFail:
            return False
        else:
            return True

    def cmdCB(self, cmd):
        """Callback function for cmdVars
        """
        alldone = [aCmd.isDone for aCmd in self.cmdVars]
        if not False in alldone: 
            print 'All Commands Finished!'
            d, self.d = self.d, None
            d.callback('success')
        elif cmd.isDone:
            # move onto the next one
            nextCmd = self.cmdIter.next()
            print 'Sending Next Cmd: ', nextCmd.cmdStr
            self.dispatcher.executeCmd(nextCmd)
        else:
            # command not done, do nothing until it is
            pass