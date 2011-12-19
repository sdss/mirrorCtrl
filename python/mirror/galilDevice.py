"""Galil Device.

This talks to a Galil motor controller, which commands actuators to move the mirror and 
reads from encoders to determine mirror position.

"""
############# imported in Agile's FW device but unused.  Need em?
import os
import sys
import traceback
#############
import time # for testing
import itertools

import Tkinter
import numpy

import TclActor
import mirror

# punk (office linux box) for testing: '172.28.191.182'
ControllerAddr = '172.28.191.182'
ControllerPort = 8000 # must match in twistedGalil.py for testing...i think

class GalilStatus(object):
    """ A class to hold the current status of the device.
    
    This is intended to be updated continuously by the device,
    and returned to the user when it is asked for
    """
    def __init__(self):
        self.clear()
        
    def clear(self):
        self.cmd = None # most recent command recieved by GalilDevice
        self.time = None # time spent on current command
        self.currentPos = None # current values on all axis
        self.desPos = None # desired values for all axis
        self.err = None # error, if present

    def genInfoStr(self):
        """ Return an info string about current status. """
        
        return 'Info'
        
class GalilDevice(TclActor.TCPDevice):
    def __init__(self, callFunc = None, actor = None):
        TclActor.TCPDevice.__init__(self,
            name = "Galil",
            addr = ControllerAddr,
            port = ControllerPort,
            callFunc = callFunc,
            actor = actor,
            cmdInfo = (),  # like agile FW device?
        )
        self.mirror = actor.mirror
        self.maxMountErr = 1 # define an array if different axes have different acceptable err
        self.currCmd = None # only 1 command can execute at a time
        self.desActPos = None # actuator length target
        # most recent commanded position, this is adjusted to reduce orientation error.
        self.cmdActPos = None  
        self.replyList = []
        self.iterNum = None
        self.iterMax = None
        self.status = GalilStatus()
        
    def handleReply(self, replyStr):
        """Handle a line of output from the device.
        Called whenever the device outputs a new line of data.
        
        Inputs:
        - replyStr  the reply, minus any terminating \n
        
        Tasks include:
        - Parse the reply
        - Manage the pending commands
        - Output data to users
        - Parse status to update the model parameters
        - If a command has finished, call the appropriate command callback
        """
        
        # check for errors '?'
        replyStr = replyStr.encode("ASCII", "ignore")
        #print 'cmd: ', self.currCmd
        # not sure what message code should be...using "d" whatever that is.
        # for now, spits everything back to user.
        self.actor.writeToUsers("d", "Galil Reply=%s" % (replyStr,), cmd = self.currCmd)
        self.replyList.append(replyStr)
        if '?' in replyStr:
            # there was an error. Does the Galil hang up on errors?
            self.actor.writeToUsers("d", "Galil Error!=%s" % (replyStr,), cmd = self.currCmd)
            # end current process
            self._clearAll()
            return
        if not replyStr.lower().endswith("ok"):
            # command not finished
            return
        else:
            if (self.iterNum < self.iterMax): # self.iter* default to None
                # iterating has been enabled, current task is a move command
                # check for err
                time.sleep(2)
                encMount = self._mountFromGalTxt(self.replyList)
                orient = self.mirror.orientFromEncoderMount(encMount, self.desOrient)
                actMount = self.mirror.actuatorMountFromOrient(orient)
                actMount = numpy.asarray(actMount, dtype=float)
                actErr = self.desActPos - actMount
                if True in (numpy.abs(actErr) > self.maxMountErr):
                    # not within acceptable error range
                    # do another move
                    self.iterNum += 1
                    # add offset to previous command
                    self.cmdActPos += actErr 
                    cmdMoveStr = self._galCmdFromMount(self.cmdActPos)
                    # send a new command
                    # note: self.currCmd is unchanged
                    self.conn.writeLine(cmdMoveStr) 
                    return  
            # command is finished
            self.actor.writeToUsers("d", "Execution Finished", cmd = self.currCmd)
            # clear current command, iterNum, replyList, desActPos ...
            self._clearAll()

            
    def newCmd(self, cmdStr, callFunc=None, userCmd=None):
        """Start a new command.
        
        Slightly changed from base class definition.
        """
        # cmdStr is pre-formatted for Galil, might be the wrong way to use cmdClass... 
        cmd = self.cmdClass(cmdStr, userCmd=userCmd, callFunc=callFunc)
        self.currCmd = cmd
        self.pendCmdDict[cmd.locCmdID] = cmd # do we need this?
       # fullCmdStr = cmd.getCmdWithID()
       # try:
            #print "Device.sendCmd writing %r" % (fullCmdStr,)
        self.conn.writeLine(cmdStr)  #from fullCmdStr
        #except Exception, e:
         #   cmd.setState(isDone=True, isOK=False, textMsg=str(e))
            
    def moveTo(self, orient, userCmd):
        """ Accepts an orientation then commands the move.
        
        Subsequent moves are commanded until an acceptable orientation
        is reached (within errors)
        """
        # if Galil is busy, abort the command
        if self.currCmd == True:
            self.actor.writeToUsers("d", "Galil is busy", cmd = self.currCmd)
            return
        # enable iteration
        self.iterNum = 0
        self.iterMax = 2
        # (user) commanded orient --> mount
        mount, adjOrient = self.mirror.actuatorMountFromOrient(orient, return_adjOrient=True)
        self.desActPos = numpy.asarray(mount, dtype=float) # the goal
        self.desOrient = numpy.asarray(adjOrient, dtype=float) # initial guess for fitter
        self.cmdActPos = numpy.asarray(mount, dtype=float) # this will change upon iteration.
        # form Galil command
        cmdMoveStr = self._galCmdFromMount(mount)
        self.newCmd(cmdMoveStr, userCmd=userCmd)
        # when move is done, check orientation from encoders
    
    def stop(self):
        """ Sends an 'ST' to the Galial, causing it to stop whatever it is doing. """
        
        self.conn.writeLine('ST;')
        self.actor.writeToUsers("d", "Galil Exection Stopped", cmd = self.currCmd)
        # clear current command
        self._clearAll()
        
    def getStatus(self):
        """ Return the Galil status to the user."""
        
        statusStr = self.status.genInfoStr()
        self.actor.writeToUsers("d", statusStr, cmd = None)
        
    def home(self, axes, userCmd):
        """ Homes the actuators defined in the axes array"""
        if self.currCmd == True:
            self.actor.writeToUsers("d", "Galil is busy", cmd = self.currCmd)
            return
        # form Galil command
        cmdMoveStr = self._galCmdForHome(axes)
        self.newCmd(cmdMoveStr, userCmd=userCmd)
        # when move is done, check orientation from encoders
                
    def _galCmdFromMount(self, mount):
        """ Converts mount information into a string command for a Galil
        
        notes: 
        The actuator mechanical/positional data needs to be defined in the
        correct order for each mirror...no way to check if that was done once
        we're here."""
        
        axes = ['A', 'B', 'C', 'D', 'E', 'F']
        cmdStr = ''
        # first set relevant axes
        # XQ#STATUS will clear all, should we do that first?
        for ind, mnt in enumerate(mount):
            # fortran code uses ';', do we want character return?
            cmdStr += axes[ind] + '=' + str(mnt) + ';' # + '\n' 
        # then command move
        cmdStr += 'XQ #MOVE;' # documentation says <CR> works, code examples seem to use ;       
        return cmdStr
        
    def _galCmdForHome(self, axes):
        """ Converts mount information into a string command for a Galil
        
        Input:
        -axes: a list of upper case letters.  like: axes = ['A', 'B', 'C', 'D', 'E', 'F']
        notes: 
        The actuator mechanical/positional data needs to be defined in the
        correct order for each mirror...no way to check if that was done once
        we're here."""
        
        cmdStr = ''
        # first set relevant axes
        # XQ#STATUS will clear all, should we do that first?
        # set each axis to 0, can be any number except MAXINT, as defined by Galil documentation.
        for axis in axes:
            cmdStr += axis + '=' + '0' + ';' # + '\n' 
        # then command home
        cmdStr += 'XQ #HOME;' # documentation says <CR> works, code examples seem to use ;       
        return cmdStr
      
    def _mountFromGalTxt(self, galTxt):
        """ Parses text returned from Galil into a mount """
        mount = numpy.random.randn(6) * 100
        return mount
             
    def _clearAll(self):
            self.currCmd = None
            self.replyList = []
            self.iterNum = None
            self.iterMax = None
            self.desActPos = None
            self.cmdActPos = None
            self.desOrient = None