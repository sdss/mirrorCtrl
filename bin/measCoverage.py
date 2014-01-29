#!/usr/bin/env python

import unittest
import coverage


cov = coverage.coverage(
	source = [
		"mirrorCtrl",
	]
)
testSuite = unittest.defaultTestLoader.discover("../tests/")
cov.start()

text_runner = unittest.TextTestRunner().run(testSuite)

cov.stop()
cov.save()
cov.html_report(directory='covhtml')
cov.erase()
