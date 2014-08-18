#!/usr/bin/env python2
"""3.5m secondary mirror controller
"""
from twisted.internet import reactor
from twistedActor import startSystemLogging
from mirrorCtrl.sec35mMirrorCtrl import Sec35mMirrorCtrl, Facility

startSystemLogging(Facility)
Sec35mMirrorCtrl()
reactor.run()
