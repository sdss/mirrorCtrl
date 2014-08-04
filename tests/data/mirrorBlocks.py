"""Construct a 2.5m and 3.5m mirror blocks and mirror objects from mir.dat files.
Uses AdjLengthLink (old style).  mir.dat files have been modified to include anti-rotation
links as Mirror objects require fully specified links (6).
"""
from tcc.base.fieldWrapper import *
import os.path
import numpy
import mirrorCtrl

class MirDat(object):

    def __init__(self):
        for mir in ['prim', 'sec', 'tert']:
            for attr in [
               "NAct",
               "NConst",
               "Type",
               "NAddStatus",
               "CtrMirZ",
               "CtrBaseZ"
            ]:
                setattr(self, mir+attr, 0)
            for attr in [
                "MinMount",
                "MaxMount",
                "MountOffset",
                "MountScale",
                "ActMirX",
                "ActMirY",
                "ActMirZ",
                "ActBaseX",
                "ActBaseY",
                "ActBaseZ",
            ]:
                setattr(self, mir+attr, [0]*6)

    @property
    def _WrapperList(self):
        raise NotImplementedError('Subclasses Should override')

    def load(self, f):
        """Load data from a file-like object
        """
        self._WrapperList.load(f)

    def loadPath(self, filePath):
        """Load data from a file specified by path
        """
        self._WrapperList.loadPath(filePath)

    def dump(self, f):
        """Dump data to a file-like object
        """
        self._WrapperList.dump(f)

    def __repr__(self):
        return self._WrapperList.dumpStr()

class MirDat35(MirDat):

    @property
    def _WrapperList(self):
        """Create WrapperList cache if it does not already exist
        """
        if hasattr(self, "__WLCache"):
            return self.__WLCache

        wrapperList = WrapperList(name="Tune", obj=self,
            wrappers = (
                DocWrapper("\nMiscellaneous Parameters\n"),

                ScalarWrapper("primNAct", dtype=int),
                ScalarWrapper("secNAct", dtype=int),
                ScalarWrapper("tertType", dtype=int),
                ScalarWrapper("secNConst", dtype=int),
                ScalarWrapper("secType", dtype=int),
                ScalarWrapper("secNAddStatus", dtype=int),

                ArrayWrapper("secMinMount", numElts=5, dtype=int),
                ArrayWrapper("secMaxMount", numElts=5, dtype=int),
                ArrayWrapper("secMountOffset", numElts=5, dtype=float),
                ArrayWrapper("secMountScale", numElts=5, dtype=float),

                ArrayWrapper("secActMirX", numElts=6, dtype=float),
                ArrayWrapper("secActMirY", numElts=6, dtype=float),
                ArrayWrapper("secActMirZ", numElts=6, dtype=float),
                ArrayWrapper("secActBaseX", numElts=6, dtype=float),
                ArrayWrapper("secActBaseY", numElts=6, dtype=float),
                ArrayWrapper("secActBaseZ", numElts=6, dtype=float),

                ScalarWrapper("tertNAct", dtype=int),
                ScalarWrapper("tertNConst", dtype=int),
                ScalarWrapper("tertNAddStatus", dtype=int),

                ArrayWrapper("tertMinMount", numElts=3, dtype=int),
                ArrayWrapper("tertMaxMount", numElts=3, dtype=int),
                ArrayWrapper("tertMountOffset", numElts=3, dtype=float),
                ArrayWrapper("tertMountScale", numElts=3, dtype=float),

                ArrayWrapper("tertActMirX", numElts=6, dtype=float),
                ArrayWrapper("tertActMirY", numElts=6, dtype=float),
                ArrayWrapper("tertActMirZ", numElts=6, dtype=float),
                ArrayWrapper("tertActBaseX", numElts=6, dtype=float),
                ArrayWrapper("tertActBaseY", numElts=6, dtype=float),
                ArrayWrapper("tertActBaseZ", numElts=6, dtype=float),
            )
        )
        self.__WLCache = wrapperList
        return self.__WLCache

class MirDat25(MirDat):
    @property
    def _WrapperList(self):
        """Create WrapperList cache if it does not already exist
        """
        if hasattr(self, "__WLCache"):
            return self.__WLCache

        wrapperList = WrapperList(name="Tune", obj=self,
            wrappers = (
                DocWrapper("\nMiscellaneous Parameters\n"),

                ScalarWrapper("primNAct", dtype=int),
                ScalarWrapper("secNAct", dtype=int),
                ScalarWrapper("secNConst", dtype=int),
                ScalarWrapper("secType", dtype=int),
                ScalarWrapper("primType", dtype=int),
                ScalarWrapper("secNAddStatus", dtype=int),
                ScalarWrapper("secCtrMirZ", dtype=float),
                ScalarWrapper("secCtrBaseZ", dtype=float),
                ArrayWrapper("secMinMount", numElts=5, dtype=int),
                ArrayWrapper("secMaxMount", numElts=5, dtype=int),
                ArrayWrapper("secMountOffset", numElts=5, dtype=float),
                ArrayWrapper("secMountScale", numElts=5, dtype=float),
                ArrayWrapper("secActMirX", numElts=6, dtype=float),
                ArrayWrapper("secActMirY", numElts=6, dtype=float),
                ArrayWrapper("secActMirZ", numElts=6, dtype=float),
                ArrayWrapper("secActBaseX", numElts=6, dtype=float),
                ArrayWrapper("secActBaseY", numElts=6, dtype=float),
                ArrayWrapper("secActBaseZ", numElts=6, dtype=float),

                ScalarWrapper("tertNAct", dtype=int),

                ScalarWrapper("primNConst", dtype=int),
                ScalarWrapper("primNAddStatus", dtype=int),

                ArrayWrapper("primMinMount", numElts=6, dtype=int),
                ArrayWrapper("primMaxMount", numElts=6, dtype=int),
                ArrayWrapper("primMountOffset", numElts=6, dtype=float),
                ArrayWrapper("primMountScale", numElts=6, dtype=float),

                ArrayWrapper("primActMirX", numElts=6, dtype=float),
                ArrayWrapper("primActMirY", numElts=6, dtype=float),
                ArrayWrapper("primActMirZ", numElts=6, dtype=float),
                ArrayWrapper("primActBaseX", numElts=6, dtype=float),
                ArrayWrapper("primActBaseY", numElts=6, dtype=float),
                ArrayWrapper("primActBaseZ", numElts=6, dtype=float),
            )
        )
        self.__WLCache = wrapperList
        return self.__WLCache

def makeMirrors(mirDat):
    mirDict = {'prim': None, 'sec': None, 'tert': None}
    for mirName in mirDict.iterkeys():
        nAct = getattr(mirDat, mirName + 'NAct')
        if nAct == 0:
            continue # no mirror of this type
        actuatorList = []
        for actNum in range(nAct):
            basePos = numpy.array([
                getattr(mirDat, mirName + 'ActBaseX')[actNum],
                getattr(mirDat, mirName + 'ActBaseY')[actNum],
                getattr(mirDat, mirName + 'ActBaseZ')[actNum],

            ])
            mirPos = numpy.array([
                getattr(mirDat, mirName + 'ActMirX')[actNum],
                getattr(mirDat, mirName + 'ActMirY')[actNum],
                getattr(mirDat, mirName + 'ActMirZ')[actNum],

            ])
            minMount = getattr(mirDat, mirName + 'MinMount')[actNum]
            maxMount = getattr(mirDat, mirName + 'MaxMount')[actNum]
            scale = getattr(mirDat, mirName + 'MountScale')[actNum]
            offset = getattr(mirDat, mirName + 'MountOffset')[actNum]
            actuatorList.append(
                mirrorCtrl.AdjLengthLink(
                    basePos,
                    mirPos,
                    minMount,
                    maxMount,
                    scale,
                    offset,
                )
            )
        fixedList = []
        for fixNum in range(nAct, 6):
            basePos = numpy.array([
                getattr(mirDat, mirName + 'ActBaseX')[fixNum],
                getattr(mirDat, mirName + 'ActBaseY')[fixNum],
                getattr(mirDat, mirName + 'ActBaseZ')[fixNum],

            ])
            mirPos = numpy.array([
                getattr(mirDat, mirName + 'ActMirX')[fixNum],
                getattr(mirDat, mirName + 'ActMirY')[fixNum],
                getattr(mirDat, mirName + 'ActMirZ')[fixNum],

            ])
            fixedList.append(
                mirrorCtrl.FixedLengthLink(
                    basePos,
                    mirPos,
                )
            )
        if getattr(mirDat, mirName + 'Type') == 3:
            # its a tip trans mirror
            ctrMirZ = getattr(mirDat, mirName + 'CtrMirZ')
            ctrBaseZ = getattr(mirDat, mirName + 'CtrBaseZ')
            mirDict[mirName] = mirrorCtrl.TipTransMirror(
                ctrMirZ = ctrMirZ,
                ctrBaseZ = ctrBaseZ,
                actuatorList = actuatorList,
                fixedLinkList = fixedList,
                encoderList = None,
                minCorrList = None,
                maxCorrList = None,
            )
        else:
            mirDict[mirName] = mirrorCtrl.DirectMirror(
                actuatorList = actuatorList,
                fixedLinkList = fixedList,
                encoderList = None,
                minCorrList = None,
                maxCorrList = None,
            )
    return mirDict

if __name__ == "__main__":
    import pickle
    # read mir.dat files
    mirDat35 = MirDat35()
    dataDir = os.path.dirname(__file__)
    print 'dataDir', dataDir
    file35 = os.path.join(dataDir, 'mir_35m.dat')
    mirDat35.loadPath(file35)
    mirDat25 = MirDat25()
    file25 = os.path.join(dataDir, 'mir_25m.dat')
    mirDat25.loadPath(file25)

    # make mirrors
    mirDict35 = makeMirrors(mirDat35)

    mirDict25 = makeMirrors(mirDat25)

    # pickle these for unit tests to load to remove this file
    # the only tcc dependency, from this package

    output = open(os.path.join(dataDir, 'mirDict35.p'), "wb")
    pickle.dump(mirDict35, output)
    output.close()
    output = open(os.path.join(dataDir, 'mirDict25.p'), "wb")
    pickle.dump(mirDict25, output)
    output.close()


