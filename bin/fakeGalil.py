#!/usr/bin/env python
"""Talk to a bench mirror with only 3 axes: same parameters as 3.5m tertiary
"""
from argparse import ArgumentParser
from twisted.internet import reactor

from mirrorCtrl.fakeGalil import FakeGalilFactory

DefaultPort = 8000

parser = ArgumentParser(description = "Start a fake Galil")
parser.add_argument("-p", "--port", type = int, default = DefaultPort,
    help = "port number (defaults to %s)" % (DefaultPort,))
parser.add_argument("-v", "--verbose", action = "store_true", help = "print input and output?")

args = parser.parse_args()

reactor.listenTCP(port = args.port, factory = FakeGalilFactory(verbose = args.verbose))
print "Starting fake Galil on port %s; verbose=%s" % (args.port, args.verbose)
reactor.run()
