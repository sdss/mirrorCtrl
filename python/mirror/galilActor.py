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
        
        Non-specified orientation values are commanded as zeros.""" 

        print 'mirr ', mirror.__name__
        if not cmd or not cmd.cmdArgs:
            raise TclActor.Command.CommandError("No orientation specified")
        try:    
            cmdArgList = numpy.asarray(cmd.cmdArgs.split(","), dtype=float)
        except Exception:
            raise TclActor.Command.CommandError('Could not convert arguments to\
                a numeric array...' % (txt, ))
        if not (0 < len(cmdArgList) < 6):
            raise TclActor.Command.CommandError("Must specify 1 to 5 orientation values, \
                got %s" % (len(cmdArgList)))
        # pad extra orientations with zeros, if commanded < 5.
        if len(cmdArgList) < 5:
            pad = numpy.zeros(5 -  len(cmdArgList))
            cmdOrient = numpy.hstack((cmdArgList, pad))
        orients = ['Piston (um)=', 'Tip X (")=', 'Tip Y (")=', 'Trans X (um)=', 'Trans Y (um)=' ]
        self.writeToUsers("d", "Commanding Move For... %s " % (self.mirror), cmd = None)
        for txt, cmdArg in itertools.izip(orients, cmdOrient):
           self.writeToUsers("d", "%s %f" % (txt, cmdArg), cmd = None)
        try:
            # convert to natural units, mm and radians
            cmdOrient = cmdOrient * numpy.array([MMPerMicron, RadPerArcSec, RadPerArcSec,
                MMPerMicron, MMPerMicron], dtype = float)
            self.galilDev.moveTo(cmdOrient, userCmd=cmd)
        except Exception, e:
            raise TclActor.Command.CommandError(str(e))
        
    
    def cmd_home(self, cmd):
        """ Home an inputed motor axis, if device isn't busy."""
        # parse into axes array, eg: axes = ['A', 'B', ...]
        axes = ['A', 'B']
        self.galilDev.home(axes, userCmd=cmd)
    
    def cmd_status(self, cmd):
        """ Get current status of Galil (asynchronous) """
        #parse
        self.galilDev.getStatus()
        
    def cmd_stop(self, cmd):
        """ Send an interrupting stop signal to Galil (asynchronous). """
        #parse
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