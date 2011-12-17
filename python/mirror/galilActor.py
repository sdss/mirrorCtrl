""" Actor for mirrors (via Galil devices).

"""
import numpy
import Tkinter

import TclActor
import mirror
import galilDevice


class GalilActor(TclActor.Actor):
    def __init__(self, mir, userPort, maxUsers = 5):
        self.galilDev = galilDevice.GalilDevice(mir, callFunc = None, actor = self)
        TclActor.Actor.__init__(self,
            userPort = userPort,
            devs = [self.galilDev],
            maxUsers = 5
        )
        
    def cmd_move(self, cmd):
        """ Move mirror to a commanded orientation, if device isn't busy. """ 
        # needs exception handling
        # how will cmd be formatted for orientation?
        # convert to numpy array and pass to device
        orient = numpy.zeros(5)
        self.galilDev.moveTo(orient, userCmd=cmd)
    
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