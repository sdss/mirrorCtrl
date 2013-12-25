#!/usr/bin/env python
"""Fake 3.5m secondary Galil
"""
from argparse import ArgumentParser
from twisted.internet import reactor

from mirrorCtrl.mirrors.mir35mSec import Mirror as mirror
from mirrorCtrl.fakeMirrorCtrlWrapper import FakeMirrorCtrlWrapper

DefaultPort = 3520

parser = ArgumentParser(description = "Start a fake Galil")
parser.add_argument("-p", "--port", type = int, default = DefaultPort,
    help = "port number (defaults to %s)" % (DefaultPort,))

args = parser.parse_args()

def stateCallback(wrapper):
    if wrapper.isReady:
        print "Fake 35mSec controller running on port", wrapper.userPort
    elif wrapper.didFail:
        print "Fake 35mSec controller error"
    elif wrapper.isDone:
        print "Fake 35mSec controller shut down"

FakeMirrorCtrlWrapper(mirror=mirror, userPort=args.port, stateCallback=stateCallback)
reactor.run()
