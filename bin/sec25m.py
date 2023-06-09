#!/usr/bin/env python2
"""SDSS 2.5m secondary mirror controller
"""
from twisted.internet import reactor
from twistedActor import startSystemLogging, startFileLogging
from mirrorCtrl.sec25mMirrorCtrl import Sec25mMirrorCtrl

startSystemLogging(Sec25mMirrorCtrl.Facility)
#startFileLogging("sec25mLog")
Sec25mMirrorCtrl()
reactor.run()
