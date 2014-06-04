#!/usr/bin/env python2
from __future__ import division, absolute_import
"""Make a mirror controller startup script for a specified mirror

Typical usage (after making sure tcc is setup):

$ sudo makeTCCStartupScript sec35m >/usr/local/bin/tcc
$ sudo chmod +x /usr/local/bin/tcc
"""
import argparse

from twistedActor import makeStartupScript

# dict of telescope name (lowercase): makeStartupScript argument dict
_DataDict = {
    "sec35m": dict(
        actorName = "sec",
        binScript = "sec35m.py",
    ),
    "tert35m": dict(
        actorName = "tert",
        binScript = "tert35m.py",
    ),
    "prim25m": dict(
        actorName = "prim",
        binScript = "prim25m.py",
    ),
    "sec25m": dict(
        actorName = "sec",
        binScript = "sec25m.py",
    ),
}

_TelescopeList = sorted(_DataDict.iterkeys())

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument(
        "telescope",
        help = "Name of telescope: one of %s" % (_TelescopeList,),
    )
    
    namespace = argParser.parse_args()
    argDict = _DataDict.get(namespace.telescope.lower())
    if argDict is None:
        argParser.error("Unrecognized telescope %r; must be one of %s" % (namespace.telescope, _TelescopeList))

    startupScript = makeStartupScript(pkgName="mirrorCtrl", **argDict)
    print startupScript