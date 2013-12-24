#!/usr/bin/env python
"""Fake 3.5m secondary Galil
"""
from argparse import ArgumentParser
from twisted.internet import reactor

from mirrorCtrl.fakeGalil import FakeGalil
from mirrorCtrl.mirrors.mir35mSec import Mirror as mirror

DefaultPort = 0

parser = ArgumentParser(description = "Start a fake Galil")
parser.add_argument("-p", "--port", type = int, default = DefaultPort,
    help = "port number (defaults to %s)" % (DefaultPort,))
parser.add_argument("-v", "--verbose", action = "store_true", help = "print input and output?")

args = parser.parse_args()

def stateCallback(server):
    print "%s is %s; port=%s" % (server.name, server.state, server.port)

FakeGalil(mirror=mirror, port=args.port, verbose=args.verbose, stateCallback=stateCallback)
reactor.run()
