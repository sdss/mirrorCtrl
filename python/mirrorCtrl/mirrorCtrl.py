from __future__ import division, absolute_import
""" Actor for mirrors (via Galil devices).
"""
import sys
import traceback

import numpy
from twistedActor import Actor, CommandError, log, UserCmd
from RO.Comm.TwistedTimer import Timer
from RO.StringUtil import strFromException

from .const import convOrient2MMRad
from .version import __version__

__all__ = ["MirrorCtrl", "runMirrorCtrl"]

DefaultMaxUsers = 5

class MirrorCtrl(Actor):
    """!Mirror controller actor
    """
    def __init__(self,
        device,
        userPort,
        maxUsers = DefaultMaxUsers,
        name = 'MirrorCtrl',
        doConnect = True,
    ):
        """!Create a Galil mirror controller actor

        @param[in] device    A Galil device from galilDevice.py
        @param[in] userPort  port on which to listen for client connections
        @param[in] maxUsers  maximum allowed simultaneous users
        @param[in] doConnect  if True then connect devices on construction
        """
        self.statusTimer = Timer() # used to queue status after stop and reset commands,
            # giving the axes time to halt before status is sent
        Actor.__init__(self,
            userPort = userPort,
            devs = [device],
            maxUsers = maxUsers,
            version = __version__,
            name = name,
            doConnect = doConnect,
            doDevNameCmds = False,
        )
        self.galil = device # easier access

    def _cancelTimers(self):
        """!Cancel all timers
        """
        self.galil.userCmdQueue.queueTimer.cancel()
        self.statusTimer.cancel()

    def processOrientation(self, orientation):
        """!Convert a user specified orientation in um and arcseconds with possibly < 5
        fields specified into an orientation of 5 values in units of radians and mm.

        @param[in] orientation  [Piston (um), [Tilt X ("), [Tilt Y ("), [Trans X (um), [Trans Y (um)]]]]]
        @return numpy.array([Piston (mm), Tilt X (rad), Tilt Y (rad), Trans X (rad), Trans Y (rad)])
        """
        orientation = numpy.hstack((orientation, numpy.zeros(5-len(orientation))))
        return convOrient2MMRad(orientation)

    def cmd_galil(self, cmd):
        """!Send an arbitrary command to the Galil mirror controller. Do not put quotes around the command.

        The Galil accepts multiple commands on one line, separated by a semicolon.
        The entire line of command should result in exactly one "OK" being printed at the end,
        e.g. either by ending with a single "XQ#..." command or with MG "OK".
        However, if neither appears to be present, then '; MG "OK"' is appended.

        Note: this replaces twistedActor.Actor's default support for direct device commands,
        because we want the command to be managed by the command queue.

        @param[in] cmd  new local user command (twistedActor.UserCmd)
        """
        if not cmd or not cmd.cmdArgs:
            raise CommandError("No command specified")
        # if this command does not contain MG "OK" or and XQ# add an MG "OK" to force
        # command completion after sending
        cmdStr = cmd.cmdArgs
        if cmdStr[0] in ["'", '"'] and cmdStr[-1] in ["'",'"']:
            #string contains quotes, take em out
            cmdStr = cmdStr[1:-1]
        log.info("galil breakthrough string: %s"%cmdStr)
        if cmdStr.endswith(";"):
            cmdStr = cmdStr[:-1]

        lastCmd = cmdStr.rsplit(";", 1)[-1].replace(" ", "")
        forceOK = True
        if lastCmd.startswith("XQ#") or lastCmd == 'MG"OK"':
            # last command is an XQ# command (all of which end by outputting OK) or explicitly outputs OK
            forceOK = False
        if forceOK:
            cmdStr += '; MG "OK"'

        try:
            self.galil.runCommand(cmd, galilCmdStr=cmdStr)
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            raise CommandError(strFromException(e))
        return True
        # cmd.setState(cmd.Done)

    def cmd_move(self, cmd):
        """!Move mirror to a commanded orientation, if device isn't busy.

        @param[in] cmd  new local user command (twistedActor.UserCmd)

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
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")

        # convert orientation to mm and radians, padding with zeros as needed
        cmdOrient = self.processOrientation(cmdArgList)
        try:
            self.galil.cmdMove(cmd, orient=cmdOrient)
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            raise CommandError(strFromException(e))

        return True

    def cmd_offset(self, cmd):
        """!Offset mirror orientation by the commanded amount, if device isn't busy

        @param[in] cmd  new local user command (twistedActor.UserCmd)

        Pass 1-5 comma seperated arguements.  Order of arguemnts corresponds to:
        [Piston (um), Tilt X ("), Tilt Y ("), Trans X (um), Trans Y (um)]

        Non-specified orientation values are commanded as zeros. When an orientation
        is specified for a non adjustable degree of freedom (eg, 3.5m tert translation),
        it is silently replaced with the constrained value (typically nearly zero).
        """
        if not cmd or not cmd.cmdArgs:
            raise CommandError("No orientation offset specified")
        try:
            cmdArgList = numpy.asarray(cmd.cmdArgs.split(","), dtype=float)
        except Exception:
            raise CommandError("Could not parse %s as a comma-separated list of floats" % (cmd.cmdArgs,))
        if not (0 < len(cmdArgList) < 6):
            raise CommandError("Must specify 1 to 5 orientation values; got %s" % (len(cmdArgList)))
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")

        currOrient = self.galil.status.orient[0:5]
        if not numpy.all(numpy.isfinite(currOrient)):
            raise CommandError("Current orientation unknown")

        # convert commanded delta- orientation to mm and radians, padding with zeros as needed
        # and add to current orientation to get commanded orientation
        cmdOrient = self.processOrientation(cmdArgList) + currOrient
        try:
            self.galil.cmdMove(cmd, orient=cmdOrient)
        except Exception as e:
            traceback.print_exc(file=sys.stderr)
            raise CommandError(strFromException(e))

        return True

    def cmd_home(self, cmd):
        """!Home specified axes (e.g. A, B, C); home all axes if none specified

        @param[in] cmd  new local user command (twistedActor.UserCmd)
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
            self.galil.cmdHome(cmd, axisList = axisList)
        except Exception as e:
            raise CommandError(strFromException(e))
        self.endWithStatus(cmd)
        return True
        
    def cmd_status(self, cmd):
        """!Show status of Galil mirror controller

        @param[in] cmd  new local user command (twistedActor.UserCmd)
        """
        # twistedActor status
        Actor.cmd_status(self, cmd)
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.galil.cmdStatus(cmd)
        except Exception as e:
            raise CommandError(strFromException(e))
        return True

    def cmd_showparams(self, cmd):
        """!Show parameters of Galil mirror controller

        @param[in] cmd  new local user command (twistedActor.UserCmd)
        """
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.galil.cmdParams(cmd)
        except Exception as e:
            raise CommandError(strFromException(e))
        return True

    def cmd_init(self, cmd):
        """!Initialize the mirror controller

        Perform all of these actions:
        - reconnect the Galil, if disconnected
        - halt motion
        - cancel the currently executing command, if any
        - clear the command queue
        - return status

        @param[in] cmd  new local user command (twistedActor.UserCmd)
        """
        try:
            if not self.galil.isConnected:
                self.cmd_connDev(cmd)
            else:
                self.galil.init(cmd)
        except Exception as e:
            raise CommandError(str(e))
        return True

    def cmd_stop(self, cmd):
        """!Deprecated alias for init
        """
        return self.cmd_init(cmd)

    def cmd_reset(self, cmd):
        """!Reset the Galil using an 'RS' command.

        Perform all of these actions:
        - reconnect the Galil, if disconnected
        - reset the Galil to its power-on state (by sending "RS")
        - cancel the currently executing command, if any
        - clear the command queue
        - return status

        @warning A home command will be required after this, as it leaves the Galil not homed
        and with no idea where it is.

        @param[in] cmd  new local user command (twistedActor.UserCmd)
        """
        if not self.galil.conn.isConnected:
            raise CommandError("Device Not Connected")
        try:
            self.galil.cmdReset(cmd)
        except Exception as e:
            raise CommandError(strFromException(e))
        self.endWithStatus(cmd)
        return True

    def endWithStatus(self, cmd, delaySec=0):
        """!Queue a status command to run after the main command
        
        @param[in] cmd  user command that is being run
        @param[in] delaySec  delay time in seconds
        """
        statusCmd = UserCmd(cmdStr="%i status" % cmd.userID)
        statusCmd.cmdVerb = "status"
        statusCmd.userID = cmd.userID
        self.statusTimer.start(delaySec, self.galil.cmdStatus, statusCmd)


def runMirrorCtrl(name, device, userPort):
    """!Start up a Galil actor without any special logging; use for one-off mirrors and unit tests

    @param[in] name  name of controller, e.g. "sec35m"; used for the log file and log entries
    @param[in] device  a twistedActor-based Galil Device (see mirrorCtrl/galilDevice.py)
    @param[in] userPort  port on which actor accepts user commands
    """
    from twisted.internet import reactor

    MirrorCtrl(
        name = name,
        device = device,
        userPort = userPort,
    )
    reactor.run()
