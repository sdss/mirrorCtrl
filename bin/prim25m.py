#!/usr/bin/env python2
"""SDSS 2.5m primary mirror controller
"""
from twisted.internet import reactor
from twistedActor import startSystemLogging
from mirrorCtrl.prim25mMirrorCtrl import Prim25mMirrorCtrl

startSystemLogging(Prim25mMirrorCtrl.Facility)
Prim25mMirrorCtrl()
reactor.run()
