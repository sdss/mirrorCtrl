#!/usr/bin/env python2
from __future__ import division, absolute_import

from twisted.trial.unittest import TestCase
from twistedActor import testUtils

testUtils.init(__file__)

from mirrorCtrl.mirrors import mir35mTert
from mirrorCtrl import MirrorDispatcherWrapper

class TestMirrorDispatcherWrapper(TestCase):
    """Test basics of MirrorDispatcherWrapper
    """
    def setUp(self):
        self.dw = MirrorDispatcherWrapper(
            mirror=mir35mTert,
        )
        return self.dw.readyDeferred
    
    def tearDown(self):
        return self.dw.close()
    
    def testSetUpTearDown(self):
        self.assertFalse(self.dw.didFail)
        self.assertFalse(self.dw.isDone)
        self.assertTrue(self.dw.isReady)


if __name__ == '__main__':
    from unittest import main
    main()
