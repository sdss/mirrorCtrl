""" Actor for mirrors (via Galil devices).
"""
__all__ = ["MirrorCtrl", "runMirrorCtrl"]

import os
import traceback
import sys

import numpy
from twistedActor import Actor, CommandError, writeToLog, startLogging, CommandQueue, UserCmd#,startGlobalLogging UserCmd, BaseCmd,
import twistedActor.log as log
from RO.Comm.TwistedTimer import Timer

from const import convOrient2MMRad


Version = 0.1

DefaultMaxUsers = 5

class MirrorCtrl(Actor):
    """Mirror controller actor
    """
    def __init__(self,
        device,
        userPort,
        maxUsers = DefaultMaxUsers,
        name = 'MirrorCtrl',
        doConnect = True,
    ):
        """Create a Galil mirror controller actor
        
        @param[in] device    A Galil device from galilDevice.py
        @param[in] userPort  port on which to listen for client connections
        @param[in] maxUsers  maximum allowed simultaneous users
        @param[in] doConnect: if True then connect devices on construction
        """
        # if LogDir is specified as an environment variable
        # begin logging to it.
        try:
            LogDir = os.environ["TWISTED_LOG_DIR"]
        except KeyError:
            pass # logging will not start
        else:
            startLogging(LogDir, "mirrorCtrl.log")
        # give the device logging capabilities
        # add a slot for a status timer, to be triggered after stop and reset commands
        self.statusTimer = Timer()
        Actor.__init__(self,
            userPort = userPort,
            devs = [device],
            maxUsers = maxUsers,
            version = Version,
            name = name,
            doConnect = doConnect,
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
                "showparams" : 1,
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
            newCmds = ['showparams'],
            queuedCmds = ['showparams'],
        )  

    def logMsg(self, msgStr):
        """Write a message string to the log.  

        @param[in] msgStr: message to be written to log
        """
        writeToLog(msgStr)


    def processOrientation(self, orientation):
        """Convert a user specified orientation in um and arcseconds with possibly < 5
        fields specified into an orientation of 5 values in units of radians and mm.
        
        @param[in] orientation: [Piston (um), [Tilt X ("), [Tilt Y ("), [Trans X (um), [Trans Y (um)]]]]]
        @return numpy.array([Piston (mm), Tilt X (rad), Tilt Y (rad), Trans X (rad), Trans Y (rad)])
        """
        orientation = numpy.hstack((orientation, numpy.zeros(5-len(orientation))))
        return convOrient2MMRad(orientation)
        
    
    def cmd_move(self, cmd):
        """Move mirror to a commanded orientation, if device isn't busy. 
        
        @param[in] cmd: new local user command (twistedActor.UserCmd)

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
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdMove(cmdOrient, userCmd=cmd))
        except Exception, e:
            traceback.print_exc(file=sys.stderr)
            raise CommandError(str(e))
            
        return True
            
    def cmd_home(self, cmd):
        """Home specified axes (e.g. A, B, C); home all axes if none specified

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        # split on , and strip leading and trailing whitespace to make a list of single axis letters
        axisList = [arg.strip() for arg in cmd.cmdArgs.split(",") if arg.strip()]
        for arg in axisList:
            if len(arg) != 1:
                raise CommandError(
                    "Could not parse %s as a comma-separated list of letters" % (cmd.cmdArgs,))
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdHome(axisList, userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        return True
            
    def cmd_status(self, cmd):
        """Show status of Galil mirror controller

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        # twistedActor status
        Actor.cmd_status(self, cmd)
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdStatus(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        else:
            # if status was queued or cancelled (not running), send 
            # a cached status
            if not cmd.isActive:
                self.dev.galil.cmdCachedStatus(cmd)
        return True
        
    def cmd_showparams(self, cmd):
        """Show parameters of Galil mirror controller

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdParams(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))
        return True
        
    def cmd_stop(self, cmd):
        """Abort any executing Galil command, put Galil in known state

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdStop(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))   
        # command a status for 1 second later (roughly 2x stopping time from max speed)
        dummyCmd = UserCmd()
        self.statusTimer.start(1., self.cmdQueue.addCmd, dummyCmd, lambda: self.cmd_staus(dummyCmd))   
        return True
    
    def cmd_reset(self, cmd):
        """Reset the Galil using an 'RS' command.

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not self.dev.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, lambda: self.dev.galil.cmdReset(userCmd=cmd))
        except Exception, e:
            raise CommandError(str(e))        
        dummyCmd = UserCmd()
        self.statusTimer.start(1., self.cmdQueue.addCmd, dummyCmd, lambda: self.cmd_staus(dummyCmd))   
        return True
          
  
def runMirrorCtrl(device, userPort):
    """Start up a Galil actor
    
    @param[in] device: a twistedActor-based Galil Device (see mirrorCtrl/galilDevice.py)
    @param[in] userPort: port on which actor accepts user commands
    """
    from twisted.internet import reactor

    Actor = MirrorCtrl(
        device = device,
        userPort = userPort,
    )    

    print "%s mirror controller starting on port %s" % (device.mirror.name, userPort,)
    reactor.run()    
