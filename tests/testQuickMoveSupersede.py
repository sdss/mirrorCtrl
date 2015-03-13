#!/usr/bin/env python2
from __future__ import division, absolute_import
"""http://twistedmatrix.com/documents/current/core/howto/trial.html

Tests communication and other behavior between the Actor and Device. Commands are dispatched using a
dispatcher.
"""

from twisted.trial.unittest import TestCase
from twisted.internet.defer import Deferred, gatherResults
from twisted.internet import reactor
from twistedActor import testUtils, log

testUtils.init(__file__)

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from opscore.actor import CmdVar

from mirrorCtrl.mirrors import mir25mSec
from mirrorCtrl import MirrorDispatcherWrapper

## speed up fake galil responses
import mirrorCtrl.fakeGalil
mirrorCtrl.fakeGalil.MaxCmdTime = 0.025 # seconds, longest any command may take
mirrorCtrl.fakeGalil.ADDNOISE = True # fake a noisy encoder reading
def showReply(msgStr, *args, **kwargs): # prints what the dispatcher sees
    print 'Keyword Reply: ' + msgStr

class CmdCallback(object):
    """Call a deferred when a command is finished
    """
    def __init__(self, deferred):
        self.deferred = deferred

    def __call__(self, cmd):
        # print "generic command callback: %r, %s" % (cmd, cmd.lastCode)
        if cmd.isDone:
            deferred, self.deferred = self.deferred, None
            deferred.callback("done")

class TestMoveSupersede(TestCase):
    """Tests for each command, and how they behave in collisions
    """
    def setUp(self):
        self.name = "mirror"
        self.dw = MirrorDispatcherWrapper(
            mirror=mir25mSec,
        )
        return self.dw.readyDeferred

    def tearDown(self):
        return self.dw.close()

    @property
    def dispatcher(self):
        """Return the actor dispatcher that talks to the mirror controller
        """
        return self.dw.dispatcher

    @property
    def fakeGalil(self):
        """Return the fake Galil (instance of FakeGalil)
        """
        return self.dw.actorWrapper.deviceWrapperList[0].controller

    @property
    def actor(self):
        return self.dw.actorWrapper.actor

    def testMoveSupersede(self):
        """Send a move command while a home command is executing
        Checks:
        1. The move command is cancelled
        2. The home command succeeds
        """
        log.info("testMoveCollide")
        d1 = Deferred()
        d2 = Deferred()
        dBoth = gatherResults([d1,d2])
        orientation1 = [-108.9522,-450.0000,320.0000,0.0000,195.0656]
        orientation2 = [0.,-450.0000,320.0000,0.0000,195.0656]
        cmdMove1 = CmdVar (
                actor = self.name,
                cmdStr = 'move ' + ', '.join([str(x) for x in orientation1]),
                callFunc = CmdCallback(d1),
            )
        cmdMove2 = CmdVar (
                actor = self.name,
                cmdStr = 'move ' + ', '.join([str(x) for x in orientation2]),
                callFunc = CmdCallback(d2),
            )
        def checkResults(cb):
            """Check results after cmdVar is done
            """
            self.assertFalse(cmdMove1.didFail) # move is superseded but doesn't fail
            self.assertFalse(cmdMove2.didFail)
        dBoth.addCallback(checkResults)
        # self.dispatcher.executeCmd(cmdMove1)
        # Timer(0.04, self.dispatcher.executeCmd, cmdMove1)
        # Timer(0.04, self.dispatcher.executeCmd, cmdMove2)
        reactor.callLater(0.04, self.dispatcher.executeCmd, cmdMove1)
        reactor.callLater(0.04, self.dispatcher.executeCmd, cmdMove2)
        # self.dispatcher.executeCmd(cmdMove)
        return dBoth


if __name__ == '__main__':
    from unittest import main
    main()
