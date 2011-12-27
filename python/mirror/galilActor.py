""" Actor for mirrors (via Galil devices).

"""
import numpy
import math
import Tkinter
import itertools

import TclActor
import mirror
import galilDevice

MMPerMicron = 1 / 1000.0        # millimeters per micron
RadPerDeg  = math.pi / 180.0    # radians per degree
ArcSecPerDeg = 60.0 * 60.0      # arcseconds per degree
RadPerArcSec = RadPerDeg / ArcSecPerDeg # radians per arcsec

ConvertOrient = numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                             MMPerMicron, MMPerMicron], dtype = float)


class GalilActor(TclActor.Actor):
    def __init__(self, mir, userPort, maxUsers = 5):
        self.mirror = mir
        self.galilDev = galilDevice.GalilDevice(callFunc = None, actor = self)
        TclActor.Actor.__init__(self,
            userPort = userPort,
            devs = [self.galilDev],
            maxUsers = 5
        )
        
        
    def cmd_move(self, cmd):
        """ Move mirror to a commanded orientation, if device isn't busy. 
        
        Pass 1-5 comma seperated arguements.  Order of arguemnts corresponds to:
        [Piston (um), Tilt X ("), Tilt Y ("), Trans X (um), Trans Y (um)]
        
        Non-specified orientation values are commanded as zeros. When an orientation
        is specified for a non adjustable degree of freedom (eg, 3.5m tert translation),
        it is silently replaced as zero.""" 

        if not cmd or not cmd.cmdArgs:
            raise TclActor.Command.CommandError("No orientation specified")
        try:    
            cmdArgList = numpy.asarray(cmd.cmdArgs.split(","), dtype=float)
        except Exception:
            raise TclActor.Command.CommandError('Could not convert arguments to\
                a numeric array. Arguments should be comma separated numbers.' % (txt, ))
        if not (0 < len(cmdArgList) < 6):
            raise TclActor.Command.CommandError("Must specify 1 to 5 orientation values, \
                got %s" % (len(cmdArgList)))
        # pad extra orientations with zeros, if not specified 5.
        if len(cmdArgList) < 5:
            pad = numpy.zeros(5 -  len(cmdArgList))
            cmdOrient = numpy.hstack((cmdArgList, pad))
        else:
            cmdOrient = cmdArgList
        # write info back
        orients = ['Piston (um) =', 'Tip X (") =', 'Tip Y (") =', 'Trans X (um) =', 'Trans Y (um) =' ]
        self.writeToUsers("i", "Commanding Move ...", cmd = cmd)
        for txt, cmdArg in itertools.izip(orients, cmdOrient):
           self.writeToUsers("i", "%s %.2f" % (txt, cmdArg), cmd = cmd)
        try:
            # convert to natural units, mm and radians
            cmdOrient = cmdOrient * ConvertOrient
            self.galilDev.moveTo(cmdOrient, userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
            
    def cmd_home(self, cmd):
        """ Home an inputed motor axis, if device isn't busy.
        
        Pass comma separated arguments A-F to specify which actuators to home.  If no arguments
        are received then all actuators are homed."""
        axesList = ['A', 'B', 'C', 'D', 'E', 'F']
        # find number of movable actuators on this mirror
        movActs = len(self.mirror.actuatorList)
        # most mirrors have fewer choices than A-F
        axesList = axesList[0:movActs]
        if not cmd.cmdArgs:
            self.writeToUsers("d", "Homing All Actuators")
        else:
            cmdArgList = cmd.cmdArgs.split(",")
            # make ascii for nice printing
            cmdArgList = [arg.encode('ascii') for arg in cmdArgList]
            # check to make sure received args are in axesList
            for arg in cmdArgList:
                # remove spaces if present
                arg = str(arg).replace(' ', '') 
                if arg not in axesList:
                    raise TclActor.Command.CommandError('Unrecognized actuator: %s.\
                    Must be comma-separated and choose from: %s (For this mirror).' % (arg, axesList))    
            # commanded actuators are all valid
            axesList = cmdArgList
            alert = "Homing Actuators: %s" % (axesList,)
            self.writeToUsers("s", alert.encode('ascii'))
        try:
            self.galilDev.home(axesList, userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
            
    def cmd_status(self, cmd):
        """ Get current status of Galil (asynchronous) """
        self.galilDev.getStatus(cmd)
        
    def cmd_showparams(self, cmd):
        """ Get current status of Galil (asynchronous) """
        self.galilDev.showParams(cmd)
        
    def cmd_stop(self, cmd):
        """ Send an interrupting stop signal to Galil (asynchronous). """
        self.galilDev.stop()
        
def runGalil(mir, userPort):
    root = Tkinter.Tk()
    print "Galil actor is starting up on port %s" % (userPort,)
    galilActor = None
  #  try:
    galilActor = GalilActor(mir=mir, userPort=userPort)
    print "Galil ICC is running on port %s" % (userPort,)
    root.mainloop()
    print "GAlil ICC is shutting down"
   # except (Exception, KeyboardInterrupt), e:
          # block cmd-C while shutting down
          # (or try to; this works on MacOS X but doesn't seem to work on nimble)
#         signal.signal(signal.SIGINT, signal.SIG_IGN)
#         traceback.print_exc(file=sys.stderr)
    #    print "galil actor is shutting down due to exception: %s" % (e,)


if __name__ == "__main__":
    runGalil()