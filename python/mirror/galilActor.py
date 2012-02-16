""" Actor for mirrors (via Galil devices).
"""
import itertools
import math
import Tkinter

import numpy
import TclActor

from .galilDevice import GalilDevice, GalilDevice35M3

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                             MMPerMicron, MMPerMicron], dtype = float)


class GalilActor(TclActor.Actor):
    def __init__(self, mir, userPort, controllerAddr, controllerPort, maxUsers = 5):
        """Create a Galil mirror controller actor
        
        Inputs:
        - mir: an instance of mirror.MirrorBase
        - userPort: port on which to listen for client connections
        - maxUsers: maximum allowed simultaneous users
        """
        self.mirror = mir
        self.galilDev = GalilDevice35M3(controllerAddr, controllerPort, actor=self)
        TclActor.Actor.__init__(self,
            userPort = userPort,
            devs = [self.galilDev],
            maxUsers = 5
        )
        
    def initialConn(self):
        """Perform initial connections.  Same as Actor Base Class method, but with the
        addition of commanding 'stop' to put the Galil in a known state upon connection.
        """
        TclActor.Actor.initialConn(self)
        # do we need a pause?
        # get device state
        initCmd = TclActor.Command.UserCmd()
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
            raise TclActor.Command.CommandError("No orientation specified")
        try:    
            cmdArgList = numpy.asarray(cmd.cmdArgs.split(","), dtype=float)
        except Exception:
            raise TclActor.Command.CommandError("Could not parse %s as a comma-separated list of floats" % (cmd.cmdArgs,))
        if not (0 < len(cmdArgList) < 6):
            raise TclActor.Command.CommandError("Must specify 1 to 5 orientation values; got %s" % (len(cmdArgList)))
        # pad extra orientations with zeros, if not specified 5.
        if len(cmdArgList) < 5:
            pad = numpy.zeros(5 -  len(cmdArgList))
            cmdOrient = numpy.hstack((cmdArgList, pad))
        else:
            cmdOrient = cmdArgList
        try:
            # convert to natural units, mm and radians
            cmdOrient = cmdOrient * ConvertOrient
            self.galilDev.cmdMove(cmdOrient, userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        return True
            
    def cmd_home(self, cmd):
        """Home specified axes (e.g. A, B, C); home all axes if none specified
        """
        # split on , and strip leading and trailing whitespace to make a list of single axis letters
        axisList = [arg.strip() for arg in cmd.cmdArgs.split(",") if arg.strip()]
        for arg in axisList:
            if len(arg) != 1:
                raise TclActor.Command.CommandError(
                    "Could not parse %s as a comma-separated list of letters" % (cmd.cmdArgs,))
        try:
            self.galilDev.cmdHome(axisList, userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        return True
        
    def cmd_log(self, cmd):
        """Prints raw (unparsed) Galil reply log to user
        """
        try:
            self.galilDev.cmdLog(userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        return True 
            
    def cmd_status(self, cmd):
        """Show status of Galil mirror controller
        """
        try:
            self.galilDev.cmdStatus(cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        return True
        
    def cmd_showparams(self, cmd):
        """Show parameters of Galil mirror controller
        """
        try:
            self.galilDev.cmdParams(cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        return True
        
    def cmd_stop(self, cmd):
        """Abort any executing Galil command
        """
        try:
            self.galilDev.cmdStop(cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))        
        return True
        
def runGalil(mir, userPort, controllerAddr, controllerPort):
    root = Tkinter.Tk()
    print "Galil actor is starting up on port %s" % (userPort,)
    galilActor = None
    galilActor = GalilActor(mir=mir, userPort=userPort, controllerAddr=controllerAddr, controllerPort = controllerPort)
    print "Galil ICC is running on port %s" % (userPort,)
    root.mainloop()
    print "Galil ICC is shutting down"
