#!/usr/bin/env python
from twisted.internet import reactor

from mirrorCtrl.mirrors.mir35mSec import Mirror as mir35mSec
from mirrorCtrl.fakeGalilDeviceWrapper import FakeGalilDeviceWrapper

def foo(wrap):
    print "wrap state changed; isReady=%s, isDone=%s, didFail=%s" % (wrap.isReady, wrap.isDone, wrap.didFail)

fakeWrapper = FakeGalilDeviceWrapper(
    mirror=mir35mSec,
    stateCallback=foo,
)

reactor.run()
