from __future__ import division, absolute_import
""" Actor for mirrors (via Galil devices).
"""
import functools
import os
import sys
import traceback

import numpy
from twistedActor import Actor, CommandError, writeToLog, CommandQueue, UserCmd
from RO.Comm.TwistedTimer import Timer

from .const import convOrient2MMRad

__all__ = ["MirrorCtrl", "runMirrorCtrl"]

Version = 0.5

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
        # add a slot for a status timer, to be triggered after stop and reset commands
        self.statusTimer = Timer()
        # set a method cmd_<devCmdName> such that any direct commands
        # are intercepted and put on the queue for tracking
        setattr(self, "cmd_%s"%device.name, self.captureDirectDevCmds)
        Actor.__init__(self,
            userPort = userPort,
            devs = [device],
            maxUsers = maxUsers,
            version = Version,
            name = name,
            doConnect = doConnect,
            doDevCmd = False,
        )
        self.galil = device # easier access
        def killFunc(killThisCmd, killedByCmd):
            writeToLog("%r queued, killing executing %r" % (killedByCmd, killThisCmd))
            # add a stop command to the queue (will be inserted in queue before killedByCmd)
            userCmd = UserCmd()
            userCmd.cmdVerb = 'stop'
            userCmd._cmdStr = 'stop (autogenerated by killFunc)'
            self.cmdQueue.addCmd(userCmd, self.galil.cmdStop)

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
            action = CommandQueue.KillRunning, # note this will also cancel a queued move
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

    def _cancelTimers(self):
        """Cancel all timers
        """
        self.cmdQueue.queueTimer.cancel()
        self.statusTimer.cancel()

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

    def captureDirectDevCmds(self, cmd):
        """This is called from the attribute set during initialization ... cmd_<devName>
        cmdArgs are sent directly to the device, and cmd is tracked on the queue in the normal
        way

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not cmd or not cmd.cmdArgs:
            raise CommandError("No command specified")
        # if this command does not contain MG "OK" or and XQ# add an MG "OK" to force
        # command completion after sending
        cmdStr = cmd.cmdArgs
        split = cmdStr.split(";")
        # split will return empty string in last list item if cmdStr ends with ";"
        lastBit = split[-1] if split[-1] else split[-2]
        forceOK = True
        if "XQ#" in lastBit.replace(" ", "") :
            forceOK = False # XQ command will force the return ok
        elif "MG" in lastBit:
            # figure out if MG "OK" was sent, if so do not force an ok
            anOK = lastBit.split("MG")[-1].strip() == '"OK"'
            if anOK:
                forceOK = False
        if forceOK:
            if not cmdStr[-1] == ";":
                cmdStr = cmdStr + ";"
            cmdStr = cmdStr + 'MG "OK"'
        try:
            self.cmdQueue.addCmd(cmd, functools.partial(self.galil.runCommand, galilCmdStr=cmdStr))
        except Exception, e:
            traceback.print_exc(file=sys.stderr)
            raise CommandError(str(e))
        return True
        cmd.setState(cmd.Done)

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
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, functools.partial(self.galil.cmdMove, orient=cmdOrient))
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
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, functools.partial(self.galil.cmdHome, axisList = axisList))
        except Exception, e:
            raise CommandError(str(e))
        return True

    def cmd_status(self, cmd):
        """Show status of Galil mirror controller

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        # twistedActor status
        Actor.cmd_status(self, cmd)
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            if self.cmdQueue.currExeCmd.cmd.isDone:
                # put a status command on the stack
                self.cmdQueue.addCmd(cmd, self.galil.cmdStatus)
            else:
                # currently executing a command, send a cached status
                self.galil.cmdCachedStatus(cmd)
        except Exception, e:
            raise CommandError(str(e))
        return True

    def cmd_showparams(self, cmd):
        """Show parameters of Galil mirror controller

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, self.galil.cmdParams)
        except Exception, e:
            raise CommandError(str(e))
        return True

    def cmd_stop(self, cmd):
        """Abort any executing Galil command, put Galil in known state

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """

        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, self.galil.cmdStop)
        except Exception, e:
            raise CommandError(str(e))
        # command a status for 1 second later (roughly 2x stopping time from max speed)
        dummyCmd = UserCmd(cmdStr="%i status" % cmd.userID)
        dummyCmd.cmdVerb = "status"
        dummyCmd.userID = cmd.userID
        self.statusTimer.start(1., self.cmdQueue.addCmd, dummyCmd, self.galil.cmdStatus)
        return True

    def cmd_reset(self, cmd):
        """Reset the Galil using an 'RS' command.

        @param[in] cmd: new local user command (twistedActor.UserCmd)
        """
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.cmdQueue.addCmd(cmd, self.galil.cmdReset)
        except Exception, e:
            raise CommandError(str(e))
        dummyCmd = UserCmd(cmdStr="%i status"%cmd.userID)
        dummyCmd.cmdVerb = "status"
        dummyCmd.userID = cmd.userID
        self.statusTimer.start(1., self.cmdQueue.addCmd, dummyCmd, self.galil.cmdStatus)
        return True


def runMirrorCtrl(name, device, userPort):
    """Start up a Galil actor

    @param[in] name: name of controller, e.g. "sec35m"; used for the log file and log entries
    @param[in] device: a twistedActor-based Galil Device (see mirrorCtrl/galilDevice.py)
    @param[in] userPort: port on which actor accepts user commands
    """
    from twisted.internet import reactor
    from twistedActor import startLogging

    # if LogDir is specified as an environment variable, begin logging to it.
    try:
        LogDir = os.environ["TWISTED_LOG_DIR"]
    except KeyError:
        pass # logging will not start
    else:
        startLogging(LogDir, name + ".log")

    MirrorCtrl(
        name = name,
        device = device,
        userPort = userPort,
    )
    reactor.run()
