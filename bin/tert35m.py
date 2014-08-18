#!/usr/bin/env python2
"""3.5m tertiary mirror controller
"""
from twisted.internet import reactor
from twistedActor import startSystemLogging
from mirrorCtrl.tert35mMirrorCtrl import Tert35mMirrorCtrl

startSystemLogging(Tert35mMirrorCtrl.Facility)
Tert35mMirrorCtrl()
reactor.run()
