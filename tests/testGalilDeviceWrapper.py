#!/usr/bin/env python2
from __future__ import division, absolute_import

from twisted.trial.unittest import TestCase
from twistedActor import testUtils

testUtils.init(__file__)

from mirrorCtrl.mirrors import mir35mTert
from mirrorCtrl import GalilDeviceWrapper

class TestGalilDeviceWrapper(TestCase):
    """Test basics of GalilDeviceWrapper
    """
    def setUp(self):
        self.dw = GalilDeviceWrapper(
            mirror=mir35mTert,
        )
        return self.dw.readyDeferred

    def tearDown(self):
        d = self.dw.close()
        return d

    def testSetUpTearDown(self):
        self.assertFalse(self.dw.didFail)
        self.assertFalse(self.dw.isDone)
        self.assertTrue(self.dw.isReady)


if __name__ == '__main__':
    from unittest import main
    main()
