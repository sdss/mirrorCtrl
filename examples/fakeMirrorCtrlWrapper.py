#!/usr/bin/env python
from twisted.internet import reactor

from mirrorCtrl.mirrors.mir35mSec import Mirror as mir35mSec
from mirrorCtrl.fakeMirrorCtrlWrapper import FakeMirrorCtrlWrapper

def foo(wrap):
    print "wrap state changed; isReady=%s, isDone=%s, didFail=%s" % (wrap.isReady, wrap.isDone, wrap.didFail)
    if wrap.isReady:
        import pdb; pdb.set_trace()

fakeWrapper = FakeMirrorCtrlWrapper(
    mirror=mir35mSec,
    stateCallback=foo,
)

reactor.run()
