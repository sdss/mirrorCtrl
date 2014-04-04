#!/usr/bin/env python2
"""Fake 3.5m secondary Galil
"""
from argparse import ArgumentParser
from twisted.internet import reactor

from mirrorCtrl.mirrors import mir35mSec
from mirrorCtrl import MirrorCtrlWrapper

DefaultPort = 3520

parser = ArgumentParser(description = "Start a fake 3.5m secondary mirror controller")
parser.add_argument("-p", "--port", type = int, default = DefaultPort,
    help = "port number (defaults to %s)" % (DefaultPort,))
parser.add_argument("-v", "--verbose", action = "store_true", help = "print input and output?")

args = parser.parse_args()

def stateCallback(wrapper):
    if wrapper.isReady:
        print "Fake 35mSec controller running on port", wrapper.userPort
    elif wrapper.didFail:
        print "Fake 35mSec controller error"
    elif wrapper.isDone:
        print "Fake 35mSec controller shut down"

MirrorCtrlWrapper(
    mirror=mir35mSec,
    userPort=args.port,
    verbose=args.verbose,
    stateCallback=stateCallback,
)
reactor.run()
