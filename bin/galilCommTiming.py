#!/usr/bin/env python2
from __future__ import absolute_import, division
"""Talk to a bench mirror with only 3 axes: same parameters as 3.5m tertiary
"""
import time
import numpy
from telnetlib import Telnet
from multiprocessing import Process, Pool

from uwBenchGalil import GalilHost, GalilPort

from twisted.internet.protocol import ClientFactory
from twisted.protocols.basic import LineReceiver
from twisted.internet import reactor
import sys

# for testing locally
GalilHost = 'tccservdev.astro.washington.edu'
GalilPort = 2011

# testing via tunnell
# GalilHost = "localhost"
# GalilPort = 16000


def cmdMirrorCtrl():
    tn = Telnet("localhost", 3532)
    time.sleep(2)
    tn.write("status\r")
    print "status"
    time.sleep(1)
    # tn.write("status\r")
    # time.sleep(.005)
    # tn.write("move 10\r")
    for move in numpy.arange(0,100,25):
        tn.write("move %i\r"%move)
        time.sleep(.0001)
    tn.close()

def hammerMirrorCtrlProc(cmdstr, period, maxIter = 10, moveStart=0):
    move = moveStart
    tn = Telnet("localhost", 3532)
    time.sleep(1)
    for i in range(maxIter):
        if cmdstr=="move":
            move += 10
            cmdstr_ = cmdstr + " %i\r"%move
        else:
            cmdstr_ = cmdstr + "\r"
        print "iter: %i cmdstr: %s"%(i, cmdstr_)
        tn.write(cmdstr_)
        time.sleep(period)
    tn.close()

def hammerMirrorCtrl():
    tn = Telnet("localhost", 3532)
    time.sleep(2)
    tn.write("status\r")
    print "status"
    time.sleep(1)
    tn.close()
    period = 10
    p = Pool(4)
    moveProc1 = p.apply_async(hammerMirrorCtrlProc, ("move", period))
    time.sleep(period/4.)
    moveProc2 = p.apply_async(hammerMirrorCtrlProc, ("move", period, 10, 200))
    time.sleep(period/4.)
    moveProc3 = p.apply_async(hammerMirrorCtrlProc, ("move", period, 10, 400))
    time.sleep(period/4.)
    moveProc3 = p.apply_async(hammerMirrorCtrlProc, ("move", period, 10, 600))
    p.close()
    p.join()



    # statusProc = Process(target=hammerMirrorCtrlProc, args=("status", period*.9))
    # moveProc1.start()
    # # moveProc1.join()
    # moveProc2.start()
    # # moveProc2.join()
    # moveProc3.start()
    # # moveProc3.join()
    # statusProc.start()
    # # statusProc.join()


def statusStop():
    tn = Telnet("localhost", 3532)
    time.sleep(2)
    tn.write("status\r")
    print "status"
    time.sleep(1)
    move = 0
    for delayTime in numpy.linspace(0.05, 0.07, 10):
        print "delay time %.4f"%delayTime
        tn.write("status\r")
        time.sleep(delayTime)
        tn.write("move %i\r"%move)
        time.sleep(delayTime)
        tn.write("stop\r")
        time.sleep(2)
        move+=10
    tn.close()

def printlines(tn, delayTime):
    print "delay time: %.2f" % delayTime
    while True:
        lineout = tn.read_until("\r", .2)
        if lineout:
            print lineout
        else:
            break
    print "------------------------------"
    print

def cmdGalil():
    tn = Telnet(GalilHost, GalilPort)
    # tn.set_debuglevel(1)
    time.sleep(1)
    print "connected"
    for delayTime in numpy.linspace(0,0.1,20):
        tn.write("XQ#STATUS\r")
        time.sleep(delayTime)
        tn.write("XQ#STOP\r")
        # tn.write("ST;MG OK")
        time.sleep(2)
        printlines(tn, delayTime)
    tn.close()

if __name__ == '__main__':
    # cmdMirrorCtrl()
    hammerMirrorCtrl()
    # statusStop()
    # cmdGalil()