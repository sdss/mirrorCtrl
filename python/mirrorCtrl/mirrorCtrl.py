""" Actor for mirrors (via Galil devices).
"""
__all__ = ["MirrorCtrl", "runMirrorCtrl"]

import itertools
import math
import os
import numpy

from twistedActor import Actor, CommandError, UserCmd, BaseCmd, writeToLog, startGlobalLogging, CommandQueue#, QueuedCommand

Version = 0.1

DefaultMaxUsers = 5

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                             MMPerMicron, MMPerMicron], dtype = float)

try:
    LogDir = os.environ["TCC_LOG_DIR"]
except KeyError:
    LogDir = None
# if LogDir is defined, start global logging
if LogDir:
    startGlobalLogging(LogDir)

class MirrorCtrl(Actor):
    def __init__(self,
        device,
        userPort,
        maxUsers = DefaultMaxUsers,
        name = 'MirrorCtrl',
    ):
        """Create a Galil mirror controller actor
        
        Inputs:
        - device    A Galil device from galilDevice.py
        - userPort  port on which to listen for client connections
        - maxUsers  maximum allowed simultaneous users
        """
        # if TCC_LOGDIR is specified as an environment variable
        # begin logging to it.
        self.logging = bool(LogDir)
        # give the device logging capabilities
        device.logMsg = self.logMsg
        Actor.__init__(self,
            userPort = userPort,
            devs = [device],
            maxUsers = maxUsers,
            version = Version,
            name = name,
        )
        def killFunc(killThisCmd):
            killThisCmd.setState(killThisCmd.Cancelling)
            # the galil device is listening for the cancelling state
            # and will take charge of fully canceling the command
            # after any necessary cleanup has happened.
        self.cmdQueue = CommandQueue(
            killFunc=killFunc,
            priorityDict = {
                "stop" : CommandQueue.Immediate,
                "reset" : CommandQueue.Immediate,
                "move" : 3,
                "home" : 3,
                "status" : 1,
                "params" : 1,
            }
        )
        self.cmdQueue.addRule(
            action = CommandQueue.KillRunning,
            newCmds = ['move'],
            queuedCmds = ['move'],
        )
        self.cmdQueue.addRule(
            action = CommandQueue.CancelNew,
            newCmds = ['move'],
            queuedCmds = ['home'],
        )
        self.cmdQueue.addRule(
            action = CommandQueue.CancelNew,
            newCmds = ['home'],
            queuedCmds = ['move'],
        )
        self.cmdQueue.addRule(
            action = CommandQueue.CancelNew,
            newCmds = ['status'],
            queuedCmds = ['status'],
        )        
        self.cmdQueue.addRule(
            action = CommandQueue.CancelNew,
            newCmds = ['params'],
            queuedCmds = ['params'],
        )  

    def logMsg(self, msgStr):
        """Write a message string to the log.  
        """
        if self.logging:
            writeToLog(msgStr, systemName=self.name, logPath=LogDir) # system adds brackets


    def processOrientation(self, orientation):
        """Convert a user specified orientation in um and arcseconds with possibly < 5
        fields specified into an orientation of 5 values in units of radians and mm.
        
        Input:
        orientation: [Piston (um), [Tilt X ("), [Tilt Y ("), [Trans X (um), [Trans Y (um)]]]]]
        
        Output:
        numpy.array([Piston (mm), Tilt X (rad), Tilt Y (rad), Trans X (rad), Trans Y (rad)])
        """
        orientation = numpy.hstack((orientation, numpy.zeros(5-len(orientation))))
        return orientation * ConvertOrient
        
    
    def cmd_move(self, cmd):
        """Move mirror to a commanded orientation, if device isn't busy. 
        
        Pass 1-5 comma seperated arguements.  Order of arguemnts corresponds to:
        [Piston (um), Tilt X ("), Tilt Y ("), Trans X (um), Trans Y (um)]
        
        Non-specified orientation values are commanded as zeros. When an orientation
        is specified for a non adjustable degree of freedom (eg, 3.5m tert translation),
        it is silently replaced with the constrained value (typically nearly zero).
        """
        if not cmd or not cmd.cmdArgs:
            raise CommandError("No orientation specified")
        try:    
            cmdArgList = numpy.asarray(cmd.cmdArgs.split(","), dtype=float)
        except Exception:
            raise CommandError("Could not parse %s as a comma-separated list of floats" % (cmd.cmdArgs,))
        if not (0 < len(cmdArgList) < 6):
            raise CommandError("Must specify 1 to 5 orientation values; got %s" % (len(cmdArgList)))
        # pad extra orientations with zeros, if not specified 5.
        cmdOrient = self.processOrientation(cmdArgList)
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdMove(cmdOrient, userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        return True
            
    def cmd_home(self, cmd):
        """Home specified axes (e.g. A, B, C); home all axes if none specified
        """
        # split on , and strip leading and trailing whitespace to make a list of single axis letters
        axisList = [arg.strip() for arg in cmd.cmdArgs.split(",") if arg.strip()]
        for arg in axisList:
            if len(arg) != 1:
                raise CommandError(
                    "Could not parse %s as a comma-separated list of letters" % (cmd.cmdArgs,))
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdHome(axisList, userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        return True
            
    def cmd_status(self, cmd):
        """Show status of Galil mirror controller
        """
        # twistedActor status
        Actor.cmd_status(self, cmd)
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdStatus(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        else:
            # if status was queued or cancelled (not running), send 
            # a cached status
            if not cmd.isActive:
                self.dev.galilDevice.cmdCachedStatus(cmd)
        return True
        
    def cmd_showparams(self, cmd):
        """Show parameters of Galil mirror controller
        """
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdParams(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        return True
        
    def cmd_stop(self, cmd):
        """Abort any executing Galil command, put Galil in known state
        """
        
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdStop(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))     
        return True
    
    def cmd_reset(self, cmd):
        """Reset the Galil using an 'RS' command.
        """
        if not self.dev.galilDevice.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galilDevice.cmdReset(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))        
        return True
          
  
def runMirrorCtrl(device, userPort):
    """Start up a Galil actor
    
    Inputs:
    device      a twistedActor-based Galil Device (see mirrorCtrl/galilDevice.py)
    userPort    port on which actor accepts user commands
    """
    from twisted.internet import reactor

    Actor = MirrorCtrl(
        device = device,
        userPort = userPort,
    )    

    print "%s mirror controller starting on port %s" % (device.mirror.name, userPort,)
    reactor.run()    
