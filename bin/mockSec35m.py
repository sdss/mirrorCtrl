#!/usr/bin/env python2
"""Fake 3.5m secondary Galil
"""
import os
from argparse import ArgumentParser
from twisted.internet import reactor

from mirrorCtrl.mirrors import mir35mSec
from mirrorCtrl import MirrorCtrlWrapper

from twistedActor import startFileLogging

try:
    LogDir = os.environ["TWISTED_LOG_DIR"]
    startFileLogging(os.path.join(LogDir, "emulate35m.log"))
except KeyError:
    # don't start logging
    pass

DefaultPort = 3520

parser = ArgumentParser(description = "Start a fake 3.5m secondary mirror controller")
parser.add_argument("-p", "--port", type = int, default = DefaultPort,
    help = "port number (defaults to %s)" % (DefaultPort,))
parser.add_argument("-v", "--verbose", action = "store_true", help = "print input and output?")

args = parser.parse_args()

MirrorCtrlWrapper(
    name = "mockSec35m",
    mirror = mir35mSec,
    userPort = args.port,
    verbose = args.verbose,
)
reactor.run()
