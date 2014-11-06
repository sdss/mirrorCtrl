#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Make a mirror controller startup script for a specified mirror

Typical usage (after making sure tcc is setup):

$ sudo makeTCCStartupScript sec35m >/usr/local/bin/tcc
$ sudo chmod +x /usr/local/bin/tcc
"""
import argparse

from twistedActor import makeStartupScript

from mirrorCtrl.sec35mMirrorCtrl import Sec35mMirrorCtrl
from mirrorCtrl.tert35mMirrorCtrl import Tert35mMirrorCtrl
from mirrorCtrl.prim25mMirrorCtrl import Prim25mMirrorCtrl
from mirrorCtrl.sec25mMirrorCtrl import Sec25mMirrorCtrl

# dict of mirror name (lowercase): makeStartupScript argument dict
_DataDict = {
    "sec35m": dict(
        actorName = "sec",
        userPort = Sec35mMirrorCtrl.UserPort,
        facility = Sec35mMirrorCtrl.Facility,
        binScript = "sec35m.py",
    ),
    "tert35m": dict(
        actorName = "tert",
        userPort = Tert35mMirrorCtrl.UserPort,
        facility = Tert35mMirrorCtrl.Facility,
        binScript = "tert35m.py",
    ),
    "prim25m": dict(
        actorName = "prim",
        userPort = Prim25mMirrorCtrl.UserPort,
        facility = Prim25mMirrorCtrl.Facility,
        binScript = "prim25m.py",
    ),
    "sec25m": dict(
        actorName = "sec",
        userPort = Sec25mMirrorCtrl.UserPort,
        facility = Sec25mMirrorCtrl.Facility,
        binScript = "sec25m.py",
    ),
}

_MirrorList = sorted(_DataDict.iterkeys())

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "mirror",
        help = "Name of mirror: one of %s" % (_MirrorList,),
    )
    
    namespace = argParser.parse_args()
    argDict = _DataDict.get(namespace.mirror.lower())
    if argDict is None:
        argParser.error("Unrecognized mirror %r; must be one of %s" % (namespace.mirror, _MirrorList))

    startupScript = makeStartupScript(pkgName="mirrorCtrl", **argDict)
    print startupScript
