#!/usr/bin/env python2
from twisted.internet import reactor

from mirrorCtrl.mirrors import mir35mSec
from mirrorCtrl.galilDeviceWrapper import GalilDeviceWrapper

def foo(wrap):
    print "wrap state changed; isReady=%s, isDone=%s, didFail=%s" % (wrap.isReady, wrap.isDone, wrap.didFail)

fakeWrapper = GalilDeviceWrapper(
    mirror=mir35mSec,
    stateCallback=foo,
)

reactor.run()
