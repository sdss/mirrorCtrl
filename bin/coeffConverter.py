#!/usr/bin/env python2
import numpy
import itertools
import glob
import os
import numpy.linalg
from collections import OrderedDict
from mirrorCtrl.mirrors import mir35mTertInfLink, mir35mTert, mir35mSecOldModel, mir35mSec
from mirrorCtrl.const import convOrient2UMArcsec, convOrient2MMRad


class FileConverter(object):
    def __init__(self, inFilename, outFilename):
        """Searches through inFileName makes orientation coefficient conversions, saves output to outfileName
        @param[in] inFilename: string, path to file to read in
        @param[in] outFilename: string, path to output file
        """
        with open(inFilename, "r") as fin:
            with open(outFilename, "w") as fout:
                line = fin.readline()
                while line:
                    if line.startswith("SecPistCoef"):
                        # do secondary mirror coefficient conversions
                        # get whole block
                        coefBlock = self.getCoefBlock(line, fin, "Sec") # will read next 4 lines from fin
                        # convert it and write it!
                        self.convertAndWrite(coefBlock, "Sec", fout)
                    elif line.startswith("TertPistCoef"):
                        # do tertiary mirror coefficient conversions
                        coefBlock = self.getCoefBlock(line, fin, "Tert")  # will read next 4 lines from fin
                        # convert it and write it!
                        self.convertAndWrite(coefBlock, "Sec", fout)
                    else:
                        # do nothing!
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
        altMatrix[:,1] = numpy.cos(altRange) # column 1 is cos term at each altitude
        altMatrix[:,2] = numpy.sin(altRange) # column 2 is sin term at each altitude
        # compute orientations at each altitude
        orientations = numpy.dot(altMatrix, oldCoefBlock)
        assert orientations.shape == (len(altRange), 5)
        for ii in range(len(altRange)):
            if mir == "Sec":
                newOrient = self.convertSecOrient(orientations[ii,:])[:5]
            else:
                assert mir == "Tert"
                newOrient = self.convertTertOrient(orientations[ii,:])[:5]
            # overwrite the new orientations with the old.
            orientations[ii,:] = newOrient
        # now find the new coefficients, using least squares
        newCoefBlock = numpy.linalg.lstsq(altMatrix, orientations)[0] # tuple is returned
        assert newCoefBlock.shape == oldCoefBlock.shape
        return newCoefBlock.T # put it back in the right shape for output.

    def convertSecOrient(self, orient):
        """Return an orientation in um and arcseconds from the input orient in um and arcseconds.
        Conversion is based on the modeled secondary mirror geometry (old vs new)
        """
        encPos = mir35mSecOldModel.encoderMountFromOrient(convOrient2MMRad(orient))
        _orient = mir35mSec.orientFromEncoderMount(encPos, initOrient=convOrient2MMRad(orient))
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
        """ Write the coefficients in coefBlock to fout
            @param[in] coefBlock: output of getCoefBlock, a numpy array
            @param[in] mir: either "Sec" or "Tert"
            @param[in] fout: file handler to which we are writing
        """
        for label, row in itertools.izip(["PistCoef", "XTiltCoef", "YTiltCoef", "XTransCoef", "YTransCoef"], coefBlock):
            numFmt = "  ".join(["%.1f"%x for x in row])
            fout.write(mir + label + "  " + numFmt + "   ! written by coeffConverter.\n")

    def convertAndWrite(self, coefBlock, mir, fout):
        """ @param[in] coefBlock: output of getCoefBlock, a numpy array
            @param[in] mir: either "Sec" or "Tert"
            @param[in] fout: file handler to which we are writing
        """
        # is any fitting required? Check for nonzero 2nd and 3rd terms in coefBlock
        if numpy.all(coefBlock[:,1:] == 0):
            # no fitting, all zeros!
            if mir == "Sec":
                newOrient = self.convertSecOrient(coefBlock[:,0])[:5]
            else:
                assert mir == "Tert"
                newOrient = self.convertTertOrient(coefBlock[:,0])[:5]
            # insert this new orientation to the coefBlock
            coefBlock[:,0] = newOrient
        else:
            # we must fit the cos/sin alt terms.
            coefBlock = self.leastSquaresFitter(coefBlock, mir)
        self._writeCoefBlock(coefBlock, mir, fout)

    def getCoefBlock(self, firstLineOfBlock, fin, mir):
        """Create a block of Coefficients, be sure that they are all present
        @param[in] firstLineOfBlock: The first line (containing the pistion coeff)
        @param[in] fin: the currently open filehandler (being read one line at a time)
        @param[in] mir: string either "Sec" or "Tert"
        """
        coefBlock = numpy.zeros((5,3))
        coefBlock[0,:] = self.coefsFromLine(firstLineOfBlock)
        for ind, check in enumerate(["XTiltCoef", "YTiltCoef", "XTransCoef", "YTransCoef"]):
            # check that the line matches what is expected
            line = fin.readline()
            assert line.startswith(mir+check)
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
    """Convert all instrument files with orientation coefficients.  New files will be written
    to a new directory.

    @param[in] fromDir: directory where the instrument files are
    @param[in] toDir: directory wehre the converted files should be written.
        They will have the same filename.
    """
    assert os.path.isdir(fromDir)
    if not os.path.isdir(toDir):
        #make it
        os.mkdir(toDir)
    # create ouput directory
    files = glob.glob(fromDir + "/*")
    for fin in files:
        baseDir,fname = os.path.split(fin)
        fout = os.path.join(toDir, fname)
        FileConverter(fin, fout)

if __name__ == "__main__":
    batchConvert(fromDir = "/Users/csayres/APO/tccdata/inst/", toDir="/Users/csayres/APO/tccdata/instConv")





