""" Actor for mirrors (via Galil devices).
"""
__all__ = ["MirrorController", "runMirrorController"]

import itertools
import math

import numpy

from twistedActor import Actor, CommandError, UserCmd

DefaultMaxUsers = 5

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                             MMPerMicron, MMPerMicron], dtype = float)

class MirrorController(Actor):
    def __init__(self,
        device,
        userPort,
        maxUsers = DefaultMaxUsers,
    ):
        """Create a Galil mirror controller actor
        
        Inputs:
        - device    A Galil device from galilDevice.py
        - userPort  port on which to listen for client connections
        - maxUsers  maximum allowed simultaneous users
        """
        Actor.__init__(self,
            userPort = userPort,
            devs = [device],
            maxUsers = maxUsers,
        )
        
    def initialConn(self):
        """Perform initial connections.  Same as Actor Base Class method, but with the
        addition of commanding 'stop' to put the Galil in a known state upon connection.
        """
        Actor.initialConn(self)
        # get device state
        initCmd = UserCmd()
        initCmd.timeLimit = 10 # give it 10 seconds before timeout
        self.cmd_stop(initCmd) # put Galil in known state   
    
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
        if len(cmdArgList) < 5:
            pad = numpy.zeros(5 -  len(cmdArgList))
            cmdOrient = numpy.hstack((cmdArgList, pad))
        else:
            cmdOrient = cmdArgList
        try:
            # convert to natural units, mm and radians
            cmdOrient = cmdOrient * ConvertOrient
            self.dev.galil.cmdMove(cmdOrient, userCmd=cmd)
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
        try:
            self.dev.galil.cmdHome(axisList, userCmd=cmd)
        except Exception, e:
            raise CommandError(str(e))
        return True
        
    def cmd_log(self, cmd):
        """Prints raw (unparsed) Galil reply log to user
        """
        try:
            self.dev.galil.cmdLog(userCmd=cmd)
        except Exception, e:
            raise CommandError(str(e))
        return True 
            
    def cmd_status(self, cmd):
        """Show status of Galil mirror controller
        """
        # twistedActor status
        Actor.cmd_status(self, cmd)
        try:
            # additional status from Galil
            self.dev.galil.cmdStatus(cmd)
        except Exception, e:
            raise CommandError(str(e))
        return True
        
    def cmd_showparams(self, cmd):
        """Show parameters of Galil mirror controller
        """
        try:
            self.dev.galil.cmdParams(cmd)
        except Exception, e:
            raise CommandError(str(e))
        return True
        
    def cmd_stop(self, cmd):
        """Abort any executing Galil command, put Galil in known state
        """
        try:
            self.dev.galil.cmdStop(cmd)
        except Exception, e:
            raise CommandError(str(e))        
        return True
    
    def cmd_reset(self, cmd):
        """Reset the Galil using an 'RS' command.
        """
        try:
            self.dev.galil.cmdReset(cmd)
        except Exception, e:
            raise CommandError(str(e))        
        return True
        
def runMirrorController(device, userPort):
    """Start up a Galil actor
    
    Inputs:
    device      a twistedActor-based Galil Device (see mirror.galilDevice)
    userPort    port on which actor accepts user commands
    """
    from twisted.internet import reactor

    Actor = MirrorController(
        device = device,
        userPort = userPort,
    )    

    print "%s mirror controller starting on port %s" % (device.mirror.name, userPort,)
    reactor.run()    
