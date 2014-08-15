from __future__ import division, absolute_import
"""Fake Galil actor wrapper.
"""
from twistedActor import ActorWrapper

from .fakeGalil import FakeGalil
from .galilDeviceWrapper import GalilDeviceWrapper
from .mirrorCtrl import MirrorCtrl

__all__ = ["MirrorCtrlWrapper"]

class MirrorCtrlWrapper(ActorWrapper):
    """A wrapper for a MirrorCtrl talking to a fake Galil

    This wrapper is responsible for starting and stopping a fake Galil and a MirrorCtrl:
    - It builds a GalilDeviceWrapper on construction, using an auto-selected port
    - It builds a MirrorCtrl when the fake Galil is ready
    - It stops both on close()

    Public attributes include:
    - deviceWrapper: the fake Galil
    - actor: the MirrorCtrl (None until ready)
    - readyDeferred: called when the actor and fake Galil are ready
      (for tracking closure use the Deferred returned by the close method, or stateCallback).
    """
    def __init__(self,
        mirror,
        userPort = 0,
        galilClass = FakeGalil,
        verbose = False,
        wakeUpHomed = True,
        stateCallback = None,
        debug = False,
        name = None,
    ):
        """Construct a MirrorCtrlWrapper that manages its fake Galil

        @param[in] userPort  port for mirror controller connections; 0 to auto-select
        @param[in] mirror  the Mirror object used by the fake Galil
        @param[in] galilClass  class of fake Galil
        @param[in] verbose  should the fake Galil run in verbose mode?
        @param[in] wakeUpHomed  should actuators be homed upon construction, or not?
        @param[in] stateCallback  function to call when connection state of hardware controller or actor changes;
            receives one argument: this actor wrapper
        @param[in] debug  print debug messages to stdout?
        @param[in] name  name of mirror controller; defaults to mirror.name
        """
        self.name = name
        self._mirror = mirror
        self.actor = None # the MirrorCtrl, once it's built
        deviceWrapper = GalilDeviceWrapper(
            mirror = mirror,
            galilClass = galilClass,
            verbose = verbose,
            wakeUpHomed = wakeUpHomed,
            debug = debug,
        )
        ActorWrapper.__init__(self,
            deviceWrapperList = [deviceWrapper],
            name = name,
            userPort = userPort,
            stateCallback = stateCallback,
            debug = debug,
        )

    def _makeActor(self):
        self.actor = MirrorCtrl(
            name = self.name or self._mirror.name,
            device = self.deviceWrapperList[0].device,
            userPort = self._userPort,
        )

