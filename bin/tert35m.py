#!/usr/bin/env python2
"""3.5m tertiary mirror controller
"""
from twisted.internet import reactor
from twistedActor import startSystemLogging
from mirrorCtrl.sec35mMirrorCtrl import Tert35mMirrorCtrl, Facility

startSystemLogging(Facility)
Tert35mMirrorCtrl()
reactor.run()
