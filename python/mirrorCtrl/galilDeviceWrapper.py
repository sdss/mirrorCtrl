from __future__ import division, absolute_import
"""Fake Galil device wrapper.
"""
from twistedActor import DeviceWrapper

from .fakeGalil import FakeGalil
from .galilDevice import GalilDevice

__all__ = ["GalilDeviceWrapper"]

class GalilDeviceWrapper(DeviceWrapper):
    """A wrapper for a GalilDevice and a fake Galil

    This wrapper is responsible for starting and stopping a fake Galil and a GalilDevice:
    - It builds a fake Galil on construction
    - It builds a GalilDevice when the fake Galil is ready
    - It stops both on close()

    Public attributes include:
    - controller: the fake Galil
    - device: the GalilDevice (None until ready)
    - readyDeferred: called when the device and fake Galil are ready
      (for tracking closure use the Deferred returned by the close method, or stateCallback).
    """
    def __init__(self,
        mirror,
        galilClass = FakeGalil,
        name = "",
        verbose = False,
        wakeUpHomed = True,
        stateCallback = None,
        port = 0,
        debug = False,
    ):
        """Construct a GalilDeviceWrapper that manages its fake Galil

        @param[in] mirror: the Mirror object used by the fake Galil
        @param[in] galilClass: class of fake Galil
        @param[in] name: name of device
        @param[in] verbose: should the fake Gail run in verbose mode?
        @param[in] wakeUpHomed: should actuators be homed upon construction, or not?
        @param[in] stateCallback: function to call when connection state of hardware controller or device changes;
            receives one argument: this device wrapper
        @param[in] port: the port for the fake Galil
        @param[in] debug: print debug messages to stdout?
        """
        self._mirror = mirror
        controller = galilClass(
            mirror = mirror,
            port = port,
            verbose = verbose,
            wakeUpHomed = wakeUpHomed,
        )
        DeviceWrapper.__init__(self,
            name = name,
            stateCallback = stateCallback,
            controller = controller,
            debug = debug,
        )
    
    def _makeDevice(self):
        port = self.port
        if port is None:
            raise RuntimeError("Controller port is unknown")
        #print "_makeDevice, port=", port
        self.device = GalilDevice(
            mirror=self._mirror,
            host="localhost",
            port=port,
        )
