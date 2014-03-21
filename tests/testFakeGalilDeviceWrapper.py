#!/usr/bin/env python
from twisted.trial.unittest import TestCase

from mirrorCtrl.mirrors import mir35mTert
from mirrorCtrl import FakeGalilDeviceWrapper

class TestFakeGalilDeviceWrapper(TestCase):
    """Test basics of FakeGalilDeviceWrapper
    """
    def setUp(self):
        self.dw = FakeGalilDeviceWrapper(
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
