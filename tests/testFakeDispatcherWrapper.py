#!/usr/bin/env python
import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from twisted.trial.unittest import TestCase
from twisted.internet import reactor

from mirrorCtrl.mirrors import mir35mTert
from mirrorCtrl.fakeDispatcherWrapper import FakeDispatcherWrapper

class TestFakeDispatcherWrapper(TestCase):
    """Test basics of FakeDispatcherWrapper
    """
    def setUp(self):
        self.dw = FakeDispatcherWrapper(
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