#!/usr/local/bin/python
"""
Combines multiple files for one device into a single file
that can then be sent to the appropriate Galil.

History:
2002-04-12 ROwen	First release.
2004-04-28 ROwen	Modified to return a flag indicating if the files are OK.
					Updated final help message.
					Bug fix: tested the wrong files if no dev-specific params.
2006-07-06 ROwen	Added load of default constants (DefConstants.py).
2008-09-08 ROwen    Changed one instance of galil -> Galil in final instructions.
"""
import fileinput
import os
import os.path
import re
import sys
import time
import checkGalilCode

# a list of files to load for each device
# each entry consists of:
# - file name (excluding directory or .gal suffix)
# - isprefix? is the name a prefix for the device name?
# - iscode? does the file contain code?
# - isoptional? is the file optional?
FileList = (
	("Parameters",   False, False, False),
	("ParamAdd",      True, False,  True),
	("DefConstants", False, False, False),
	("Constants",     True, False, False),
	("Program",      False, True,  False),
	("ProgAdd",       True, True,   True),
)
FileSuffix = ".gal"

def combineGalilCode(devName, dirName = None, outName = None):
	"""Checks the code for a device and combines it into a single file,
	with blank lines and comments missing.
	If outName is not specified, writes to "Combined <devName> <currDate>.gal"
	in the specified directory.
	"""
	if not outName:
		outName = "Combined " + devName + time.strftime(" %Y-%m-%d", time.localtime()) + ".gal"
		if dirName:
			outName = os.path.join(dirName, outName)
	print "Generating a list of files for device %r" % (devName,)
	fileList, codeList = getFileNames(devName, dirName)
	print "Checking the program files"
	cc = checkGalilCode.CodeChecker()
	fileOK = cc.checkFileSet(codeList)
	if not fileOK:
		print "Error in files; giving up"
		sys.exit(0)
	print "Writing the combined code"
	procGalilCode(fileList, outName)
	print "\nThe combined code was written to file: %r" % (outName,)
	print """
The next steps are:
- Disable the motors attached to the Galil. This step is vital
  when loading a Galil that is not configured for your motors,
  for example a Galil that has been repaired, reset to its factory defaults
  or configured for some other device.
- Upload this file to the Galil; any errors must be fixed before continuing
- Test the Galil with at least the following commands:
  XQ#SHOWPAR
  XQ#STATUS
When you are sure it works, save the new parameters, variables and program
to the flash memory using the following Galil commands:
  BN
  BV
  BP
"""

def getFileNames(devName, dirName = None):
	"""Returns the set of files (in order) required to fully upload
	code to the named device.
	Inputs:
	- devName: name of device, e.g. "SDSS M2"
	- dirName: directory containing code files (current directory if omitted)
	
	Returns two lists
	- fileList: a list of all file paths
	- codeList: a list of paths of files containing code
	"""
	fileList = []
	codeList = []
	for basicName, isprefix, iscode, isoptional in FileList:
		if isprefix:
			fileName = basicName + " " + devName + FileSuffix
		else:
			fileName = basicName + FileSuffix
		if dirName:
			fileName = os.path.join(dirName, fileName)
		if os.path.exists(fileName):
			fileList.append(fileName)
			if iscode:
				codeList.append(fileName)
		elif not isprefix:
			raise RuntimeError, "File %r missing; bad directory" % (fileName,)
		elif not isoptional:
			raise RuntimeError, "File %r missing; bad device name or directory" % (fileName,)
			
	return fileList, codeList

def procGalilCode(fileList, outName=None):
	"""Combines a set of Galil source code files into a single file,
	skipping blank lines and comment lines.
	If outName is not specified, writes to stdout.
	"""
	if outName:
		outFile = open(outName, "w")
	else:
		outFile = sys.stdout

	try:
		for line in fileinput.input(fileList):
			# strip leading and trailing whitespace, including final \n
			line = line.strip()
			if len(line) == 0:
				# skip blank lines
				continue
			elif line.startswith("NO"):
				# skip comment lines
				continue
			outFile.write(line)
			outFile.write("\n")
	finally:
		outFile.close()


if __name__ == "__main__":
	debug = False
	if debug:
		testList = ("SDSS M1", "SDSS M2", "35m M2", "Bad Device")
		for testName in testList:
			try:
				fileList, codeList = getFileNames(testName)
				print "Device %r uses files: %r" % (testName, fileList)
			except Exception, e:
				print "Failed on device %r: %s" % (testName, e)
	else:
		if len(sys.argv) not in (2, 3, 4):
			print "Combines and checks all Galil code files for a given device."
			print "To use: combineGalilCode devname [dir [outfilename]]"
		else:
			combineGalilCode(*sys.argv[1:])
