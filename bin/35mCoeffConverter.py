#!/usr/bin/env python2
import itertools
import glob
import os
import time
import re
import shutil

import numpy
import numpy.linalg
from matplotlib import pyplot as plt

from mirrorCtrl.mirrors import mir35mTertInfLink, mir35mTert, mir35mSecOldModel, mir35mSec
from mirrorCtrl.const import convOrient2UMArcsec, convOrient2MMRad

def orientationPlotter(oldOrientations, newOrientations, oldCoefBlock, newCoefBlock, altitudes):
    # coefBlock should be 3x5
    panelNames = ["pistion", "x tilt", "y tilt", "x trans", "y trans"]
    fig = plt.figure(figsize=(8,10))
    for ii in range(5):
        ax = fig.add_subplot(5,1, ii+1)
        ax.plot(numpy.degrees(altitudes), oldOrientations[:,ii], '.r', label = "old orient", alpha=0.5)
        ax.plot(numpy.degrees(altitudes), newOrientations[:,ii], '.b', label = "new orient", alpha=0.5)
        ax.plot(numpy.degrees(altitudes), oldCoefBlock[0,ii] + numpy.sin(altitudes)*oldCoefBlock[1,ii] + numpy.cos(altitudes)*oldCoefBlock[2,ii], 'r', label="old fit")
        ax.plot(numpy.degrees(altitudes), newCoefBlock[0,ii] + numpy.sin(altitudes)*newCoefBlock[1,ii] + numpy.cos(altitudes)*newCoefBlock[2,ii], 'b', label="new fit")
        ax.set_ylabel(panelNames[ii])
        if ii==0:
            ax.legend()
    ax.set_xlabel("altitude")
    plt.show(block=True)

def convertSecOrientToNew(orient):
    """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
    Conversion is based on the modeled secondary mirror geometry (old vs new)
    """
    encPos = mir35mSecOldModel.encoderMountFromOrient(convOrient2MMRad(orient))
    _orient = mir35mSec.orientFromEncoderMount(encPos[:5], initOrient=convOrient2MMRad(orient))
    newOrient = convOrient2UMArcsec(_orient)
    return newOrient

def convertTertOrientToNew(orient):
    """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
    Conversion is based on the modeled secondary mirror geometry (old vs new)
    """
    encPos = mir35mTertInfLink.encoderMountFromOrient(convOrient2MMRad(orient))
    _orient = mir35mTert.orientFromEncoderMount(encPos[:3], initOrient=convOrient2MMRad(orient))
    newOrient = convOrient2UMArcsec(_orient)
    return newOrient

def convertSecOrientToOld(orient):
    """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
    Conversion is based on the modeled secondary mirror geometry (old vs new)
    """
    encPos = mir35mSec.encoderMountFromOrient(convOrient2MMRad(orient))
    _orient = mir35mSecOldModel.orientFromEncoderMount(encPos[:5], initOrient=convOrient2MMRad(orient))
    newOrient = convOrient2UMArcsec(_orient)
    return newOrient

def convertTertOrientToOld(orient):
    """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
    Conversion is based on the modeled secondary mirror geometry (old vs new)
    """
    encPos = mir35mTert.encoderMountFromOrient(convOrient2MMRad(orient))
    _orient = mir35mTertInfLink.orientFromEncoderMount(encPos[:3], initOrient=convOrient2MMRad(orient))
    newOrient = convOrient2UMArcsec(_orient)
    return newOrient

class FileConverter(object):
    def __init__(self, inPath, outPath):
        """Searches through inFileName makes orientation coefficient conversions, saves output to outfileName
        @param[in] inPath: string, path to file to read in
        @param[in] outPath: string, path to output file
        """
        fileName = os.path.split(inPath)[1]
        convertedRE = re.compile(r"coeffs converted.+ coeffconverter")

        hasMirCoeffs = False
        with open(inPath, "r") as fin:
            for line in fin:
                lowLine = line.strip().lower()
                if not lowLine:
                    continue
                if convertedRE.search(lowLine):
                    print "%s has already been converted; copying file unchanged" % (fileName,)
                    shutil.copyfile(inPath, outPath)
                    return
                if lowLine.startswith("secpistcoef") or lowLine.startswith("tertpistcoef"):
                    hasMirCoeffs = True
        if not hasMirCoeffs:
            print "%s has no mirror coeffs; copying file unchanged" % (fileName,)
            shutil.copyfile(inPath, outPath)
            return

        with open(inPath, "r") as fin:
            with open(outPath, "w") as fout:
                print "%s conversion begins" % (fileName,)
                # use this clumsy form of iteration so convertAndWrite can read additional lines
                line = fin.readline()
                while line:
                    lowLine = line.strip().lower()
                    if lowLine.startswith("secpistcoef"):
                        # convert secondary mirror coefficient
                        coefBlock = self.getCoefBlock(line, fin, "Sec") # will read next 4 lines from fin
                        self.convertAndWrite(coefBlock, "Sec", fout)
                    elif lowLine.startswith("tertpistcoef"):
                        # convert tertiary mirror coefficients
                        coefBlock = self.getCoefBlock(line, fin, "Tert")  # will read next 4 lines from fin
                        self.convertAndWrite(coefBlock, "Tert", fout)
                    else:
                        # copy line unconverted
                        fout.write(line)
                    line = fin.readline()

    def leastSquaresFitter(self, coefBlock, mir):
        """Used to determine a new coefBlock (from the old) for a given mirror
        where sine and cosine of altitude terms are present
        @param[in] coefBlock: a 5x3 numpy array
        @param[in] mir: either "Sec" or "Tert", whichever mirror we're fitting

        note: based on linear algebra strategy,
        minimizing norm(Ax-b) (see Trefethen and Bau - Numerical Linear Algebra pg 88)
        or see documentation for numpy.linalg.lstsq.
        """
        oldCoefBlock = coefBlock.T # transpose so that dot works (must be 3x5)
        altRange = numpy.linspace(0,numpy.pi/2.,100) # 100 points equally spaced from 0 to 90 degrees alt
        altMatrix = numpy.ones((len(altRange), 3)) # matrix of altitude terms
        altMatrix[:,1] = numpy.sin(altRange) # column 1 is cos term at each altitude
        altMatrix[:,2] = numpy.cos(altRange) # column 2 is sin term at each altitude
        # compute orientations at each altitude
        oldOrientations = numpy.dot(altMatrix, oldCoefBlock)
        orientations = numpy.zeros(oldOrientations.shape)
        assert orientations.shape == (len(altRange), 5)
        for ii in range(len(altRange)):
            if mir == "Sec":
                newOrient = self.convertSecOrient(oldOrientations[ii,:])[:5]
            else:
                assert mir == "Tert"
                newOrient = self.convertTertOrient(oldOrientations[ii,:])[:5]
            # overwrite the new orientations with the old.
            orientations[ii,:] = newOrient
        # now find the new coefficients, using least squares
        newCoefBlock = numpy.linalg.lstsq(altMatrix, orientations)[0] # tuple is returned
        assert newCoefBlock.shape == oldCoefBlock.shape
        orientationPlotter(oldOrientations, orientations, oldCoefBlock, newCoefBlock, altRange)
        return newCoefBlock.T # put it back in the right shape for output.

    def convertSecOrient(self, orient):
        """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
        Conversion is based on the modeled secondary mirror geometry (old vs new)
        """
        encPos = mir35mSecOldModel.encoderMountFromOrient(convOrient2MMRad(orient))
        _orient = mir35mSec.orientFromEncoderMount(encPos[:5], initOrient=convOrient2MMRad(orient))
        newOrient = convOrient2UMArcsec(_orient)
        return newOrient

    def convertTertOrient(self, orient):
        """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
        Conversion is based on the modeled secondary mirror geometry (old vs new)
        """
        encPos = mir35mTertInfLink.encoderMountFromOrient(convOrient2MMRad(orient))
        _orient = mir35mTert.orientFromEncoderMount(encPos[:3], initOrient=convOrient2MMRad(orient))
        newOrient = convOrient2UMArcsec(_orient)
        return newOrient

    def _writeCoefBlock(self, coefBlock, mir, fout):
        """Write the coefficients in coefBlock to fout
            @param[in] coefBlock: output of getCoefBlock, a numpy array
            @param[in] mir: either "Sec" or "Tert"
            @param[in] fout: file handler to which we are writing
        """
        for label, row in itertools.izip(["PistCoef", "XTiltCoef", "YTiltCoef", "XTransCoef", "YTransCoef"], coefBlock):
            numFmt = "  ".join(["%.1f"%x for x in row])
            fout.write(mir + label + "  " + numFmt + "\n")

    def convertAndWrite(self, coefBlock, mir, fout):
        """Convert a set of coefficients and write the results

        @param[in] coefBlock: output of getCoefBlock, a numpy array
        @param[in] mir: either "Sec" or "Tert"
        @param[in] fout: file handler to which we are writing
        """
        print "  convert %s coeffs" % (mir,)
        if numpy.all(coefBlock[:,1:] == 0):
            # the sine and cosine coeffs are zero; just convert the constant coeffs
            if mir == "Sec":
                newOrient = self.convertSecOrient(coefBlock[:,0])[:5]
            else:
                assert mir == "Tert"
                newOrient = self.convertTertOrient(coefBlock[:,0])[:5]
            # insert this new orientation to the coefBlock
            coefBlock[:,0] = newOrient
        else:
            # fit the sine and cosine terms
            print "  using least squares fitter; please wait"
            coefBlock = self.leastSquaresFitter(coefBlock, mir)
        self._writeCoefBlock(coefBlock, mir, fout)
        currDateStr = time.strftime("%Y-%m-%d", time.localtime())
        fout.write("! %s coeffs converted from the old TCC's mirror model by coeffConverter %s.\n" % \
            (mir, currDateStr))

    def getCoefBlock(self, firstLineOfBlock, fin, mir):
        """Create a block of Coefficients, be sure that they are all present

        @param[in] firstLineOfBlock: The first line (containing the pistion coeff)
        @param[in] fin: the currently open filehandler (being read one line at a time)
        @param[in] mir: string either "Sec" or "Tert"
        """
        coefBlock = numpy.zeros((5,3))
        coefBlock[0,:] = self.coefsFromLine(firstLineOfBlock)
        for ind, check in enumerate(["xtiltcoef", "ytiltcoef", "xtranscoef", "ytranscoef"]):
            # check that the line matches what is expected
            line = fin.readline()
            assert line.strip().lower().startswith((mir+check).lower())
            coefBlock[ind+1,:] = self.coefsFromLine(line)
        return coefBlock

    def coefsFromLine(self, line):
        """Return a numpy 3 element array from a line containing the coeffs and ignoring comments
        """
        line = line.split("!")[0] # throw away anything after a comments
        line = line.split("Coef")[1] # just keep numbers
        numArray = line.split() # split on whitespace, numbers remain
        numArray = [float(x) for x in numArray]
        return numpy.asarray(numArray)

def batchConvert(fromDir, toDir):
    """Convert all instrument files with orientation coefficients.

    @param[in] fromDir: directory where the instrument files are
    @param[in] toDir: directory where the converted files should be written.
        They will have the same filename.
    """
    assert os.path.isdir(fromDir)
    print "Converting 3.5m inst files from %r to %r" % (fromDir, toDir)
    if not os.path.isdir(toDir):
        print "Creating output dir %r" % (toDir,)
        os.mkdir(toDir)

    files = glob.glob(fromDir + "/*")
    for inPath in files:
        fileName = os.path.split(inPath)[1]
        outPath = os.path.join(toDir, fileName)
        FileConverter(inPath, outPath)

if __name__ == "__main__":
    instDir = os.path.join(os.environ["TCC_DATA_DIR"], "instWithOldMirCoeffs")
    batchConvert(fromDir = instDir, toDir="convertedInstFiles")





