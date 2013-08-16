#!/usr/local/bin/python
"""
CheckGalilCode checks Galil code for various common errors
and reports on resource usage.

History:
2002-04-10 v1 R. Owen. First python version (based on a perl version).
2002-04-10 v2 R. Owen. Supports checking multiple files that make up a set of code;
	added interactive help.
2002-04-12 R. Owen. Refactored the code to handle fatal errors better and print less verbiage.
2002-08-13 R. Owen. Bug fixes: was mis-reporting line numbers for errors (off by 2x)
	and not detecting all end-of-blocks. Hid all constants and class LineInfo with leading underscore.
2004-03-31 ROwen	Modified to quit if the files contain an error.
"""
import sys
import re

# Constants specific to the Galil
_MaxCodeLines = 999
_MaxLineLen = 79  # excluding terminating \n
_MaxLabels = 254
_MaxLabelLen = 8
_DesiredStartLabel = "#LCLNUP"

# Pre-compiled regular expressions
_DLRE = re.compile(r"(^|;) *DL *(?P<label>#[A-Z][A-Z0-9]*)?")
_BlockEndRE = re.compile(r"([^;]*;)* *(JP|EN|RE|RI)[^;]*(; *NO)?[^;]*$")

_BadREList = (
	(_DLRE, "extra download command DL"),
	(re.compile(r".*[^ ].*\\"), "embedded backslash (ends download early)"),
	(re.compile(r"([^;]*;)* *NO.*;"),"comment containing a semicolon"),
)

_LabelRE = re.compile(r"^(#[A-Z][A-Z0-9]*)")
_LabelRefRE = re.compile(r".+(#[A-Z][A-Z0-9]*)")

_Debug = 0 # controls whether extra debugging information is printed

class CodeChecker:
	"""Class to check code (program statements) in a set of Galil code files.
	Specifically, checks:
	- Every statement label is unique
	- Every reference to a statement label refers to a label that exists
	- There are not too many statement labels
	- There are not too many lines of code (program statements)
	"""
	def __init__(self, fileNames=None, verbose=0):
		"""Creates a code checker and optionally checks a list of files.
		The files must be specified in the order in which they are to be uploaded.
		
		Inputs:
		- fileNames: list of file names, in the order in which they are to be uploaded
		- verbose: if true, status (e.g. # of labels used) is output for each file;
			otherwise status is only output after all files are checked
		"""
		self.fileName = ""
		self.fileNum = 0
		self.lineNum = 0
		self.codeLineNum = 0
		
		self.codeStart = None # line info about start of code (DL command)
		self.blockEnd = 0 # true if this line is (or the last nonblank line was)
			# the end of a block of code
			# A blank line may only follow the end of a code block
		
		# a dictionary of statement labels:
		# - keys are labels (with initial "#" character)
		# - items are _LineInfo objects
		self.labelDict = {}
		
		# a dictionary of statement label references:
		# - keys are labels (with initial "#" character)
		# - items are a list of one or more _LineInfo objects (one per reference)
		self.labelRefDict = {}
		
		if fileNames:
			self.checkFileSet(fileNames, verbose)
	
	def checkFileSet(self, fileNames, verbose=False):
		"""Check a set of files. Returns True if all files OK, False otherwise.
		"""
		self.filesOK = True
		for fileName in fileNames:
			locVerbose = verbose or (fileName == fileNames[-1]) or _Debug
			self._checkFile(fileName, locVerbose)
		return self.filesOK
	
	def _checkFile(self, fileName, verbose=0):
		"""Check a single file, combining the results with data
		from any previous files that were checked (including
		the list of files specified in the call to __init__, if any).
		
		Inputs:
		- fileName: name of file to check
		- verbose: if true, status (e.g. # of statement labels) is printed after checking
		"""
		self.fileName = fileName
		self.fileNum += 1

		print "\nChecking code in file %d: %s" % (self.fileNum, fileName)
		
		inFile = open(fileName, 'r')
		try:
			self.lineNum = 0
			for self.line in inFile:
				self.lineNum += 1
				self._checkLine()
		finally:
			inFile.close()
	
		# make sure each label reference points to an existing label
		for label in self.labelRefDict.keys():
			if label not in self.labelDict:
				self._error("reference to nonexistent statement label %s" % (label))
				for linfo in self.labelRefDict[label]:
					print " %s" % linfo
		
		if verbose:
			print
		if self.codeLineNum > _MaxCodeLines:
			self._error("There are %d lines of code out of %d allowed" % (self.codeLineNum, _MaxCodeLines))
		elif verbose:
			print        "There are %d lines of code out of %d allowed" % (self.codeLineNum, _MaxCodeLines)
		nLabels = len(self.labelDict)
		if nLabels > _MaxLabels:
			self._error("There are %d statement labels out of %d allowed" % (nLabels, _MaxLabels))
		elif verbose:
			print        "There are %d statement labels out of %d allowed" % (nLabels, _MaxLabels)
	
	def _checkLine(self):
		"""Checks the current line of data.
		"""
		strippedLine = self.line.strip()

		# if code start found (DL command), handle it (and no further processing necessary)
		dlMatch = _DLRE.match(self.line)
		if dlMatch:
			self._handleCodeStart()
			return
		
		# if code end found (\), handle it
		# this looks for the obviously intentional case;
		# embedded backslashes are handled later and are assumed to be unintentional errors
		if strippedLine.startswith("\\"):
			self.codeStart = None
			return
		
		# blank lines are OK if they follow the end of a code block
		# (and they need no further checking, in any case)
		if len(strippedLine) == 0:
			if self.codeStart and not self.blockEnd:
				self._error("blank line appears within a block of code\n %s" % (self.currLineInfo(),))
			return

		# pure comments need no further checking
		if strippedLine.startswith("NO"):
			return

		# count lines, excluding blank lines and pure comments
		if self.codeStart:
			self.codeLineNum += 1
		
		# set end of block flag
		self.blockEnd = (_BlockEndRE.match(self.line) != None)
			
		# check line against all invalid regular expressions
		for regex, errStr in _BadREList:
			if regex.match(self.line):
				self._error("%s:\n %s" % (errStr, self.currLineInfo()))

		# make sure line is short enough
		if len(self.line) > 1 + _MaxLineLen:
			self._error("line longer than %d characters:\n %s" % (_MaxLineLen, self.currLineInfo()))
		
		# delete embedded comments (for counting purposes)
		strippedLine = re.sub(r"; *NO[^;]*", "", strippedLine)

		# delete strings (for counting purposes)
		strippedLine = re.sub(r'"[^"]*"', "", strippedLine)
		
		# count labels and check for duplicates
		labelMatch = _LabelRE.match(strippedLine)
		if labelMatch:
			if not self.codeStart:
				self._error("statement label found outside a code block %r" % (label,))
			label = labelMatch.group(1)
			if label in self.labelDict:
				self._error("duplicate statement label %r" % (label,))
				print " First declared in %s" % self.labelDict[label]
				print " Duplicate in %s" % self.currLineInfo()
			else:
				if len(label) > _MaxLabelLen:
					self._error("statement label %r has more than %d characters:\n %s" \
						% (label, _MaxLabelLen, self.currLineInfo()))
				self.labelDict[label] = self.currLineInfo()

		# count label references
		labelRefMatch = _LabelRefRE.match(strippedLine)
		if labelRefMatch:
			label = labelRefMatch.group(1)
			currValue = self.labelRefDict.get(label, [])
			newValue = currValue.append(self.currLineInfo())
			self.labelRefDict.setdefault(label, []).append(self.currLineInfo())

	def currLineInfo(self):
		"""Returns a _LineInfo object for the current line of code."""
		return _LineInfo(self.fileName, self.lineNum, self.line, self.codeLineNum)

	def _handleCodeStart(self):
		"""Call whenever you read a line that starts a block of program statements,
		i.e. a DL command. If the DL command includes a statement label,
		purges information about any code that will be replaced.
		"""
		dlMatch = _DLRE.match(self.line)
		if not dlMatch:
			raise RuntimeError, "Fatal error: _handleCodeStart called on a non-DL line\n %s" % (self.getLineInfo())
		startLabel = dlMatch.groupdict()["label"]

		if self.codeStart:
			self._error("duplicate DL command found")
			print " First found in", self.codeStart
			print " Duplicate in ", self.currLineInfo()
			return

		self.codeStart = self.currLineInfo()
		
		if not startLabel:
			# code starts at the beginning;
			# this is only allowed if no code has already been downloaded
			if self.codeLineNum > 0:
				raise RuntimeError, "Error: DL with no starting label found after some code already found\n %r" % (self.currLineInfo())
			return
			
		# code does not start at the beginning
		# make sure label already exists
		if not self.labelDict.has_key(startLabel):
			raise RuntimeError, """Download starts at label %r, but no such label found:
 %s
Perhaps you have not specified all necessary files in the correct order?""" % (startLabel, self.currLineInfo())
		
		# find line number at existing label
		# and flush all existing references that are at or after it
		oldLastLineInfo = self.labelDict[startLabel]
		oldLastLineNum = oldLastLineInfo.lineNum
		self.codeLineNum = oldLastLineInfo.codeLineNum
		if startLabel != _DesiredStartLabel:
			self._error("supplementary code starts at invalid label %r; should be %r\n %s" \
				% (startLabel, _DesiredStartLabel, self.currLineInfo()))
		elif _Debug:
			print "File %r contains supplementary code starting at the usual label %r" % (self.fileName, startLabel)
		
		# update self.labelDict and self.labelRefDict
		# I create new ones instead of deleting items from old ones
		# because deletion while iterating is tricky
		newLabelDict = {}
		for label, linfo in self.labelDict.iteritems():
			if linfo.lineNum <= oldLastLineNum:
				# print "retaining label %r: %s" % (label, linfo)
				newLabelDict[label] = linfo
		self.labelDict = newLabelDict
		
		newLabelRefDict = {}
		for label, linfoList in self.labelRefDict.iteritems():
			for linfo in linfoList:
				if linfo.lineNum <= oldLastLineNum:
					# print "retaining reference to label %r: %s" % (label, linfo)
					newLabelRefDict.setdefault(label, []).append(linfo)
		self.labelRefDict = newLabelRefDict
		if _Debug:
			print "Purge of existing code finished, leaving %d lines of code and %d statement labels" % (self.codeLineNum, len(self.labelDict))
	
	def _error(self, errMsg):
		"""Call to report an error.
		Prints the message and clears the filesOK flag.
		"""
		self.filesOK = False
		print "Error:", errMsg


class _LineInfo:
	"""Keeps track of a line of code, including:
	- fileName: the name of the file
	- lineNum: the line number in the file (including non-code lines such as header, comments and blank lines)
	- line: the line of code itself (including final \n)
	- codeLineNum: line number in the code, excluding header, comments and blank lines
	  (i.e. the line number when the code is loaded into the Galil)
	"""
	def __init__(self, fileName, lineNum, line, codeLineNum = None):
		self.fileName = fileName
		self.lineNum = lineNum
		self.line = line
		self.codeLineNum = codeLineNum
	
	def __str__(self):
		return "file %r, line %d: %s" % (self.fileName, self.lineNum, self.line[:-1])
	
	def __repr__(self):
		return "_LineInfo(%r, %r, %r, %r)" %  (self.fileName, self.lineNum, self.line, self.codeLineNum)

if __name__ == "__main__":
	# process each file in turn
	if len(sys.argv) <= 1 or sys.argv[1].startswith("-"):
		print "Checks a set of Galil code files for a single device; the files must be specified in order."
		print "To use: checkGalilCode file1 [file2...]"
	else:
		CodeChecker(sys.argv[1:])
