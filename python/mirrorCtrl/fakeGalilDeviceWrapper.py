#!/usr/bin/env python
"""Fake Galil device wrapper.
"""
import sys

from twisted.internet.defer import Deferred
from RO.AddCallback import safeCall

from .fakeGalil import FakeGalil
from .galilDevice import GalilDevice

__all__ = ["FakeGalilDeviceWrapper"]

class FakeGalilDeviceWrapper(object):
    """A wrapper for a GalilDevice and a fake Galil
    
    This wrapper is responsible for starting and stopping a fake Galil and a GalilDevice:
    - It builds the fake Galil on construction
    - It builds device = GalilDevice when the fake Galil is ready
    - It stops both on close()
    
    Public attributes include:
    - hardwareController: the fake Galil (the name suggests this could be a general-purpose device wrapper)
    - device: the GalilDevice (None until ready)
    - readyDeferred: called when the device and fake Galil are ready
      (for tracking closure use the Deferred returned by the close method, or stateCallback).
    """
    def __init__(self,
        mirror,
        galilClass = FakeGalil,
        wakeUpHomed = True,
        stateCallback = None,
    ):
        """Construct a FakeGalilDeviceWrapper that manages its fake Galil

        @param[in] mirror: the Mirror object used by the fake Galil
        @param[in] galilClass: class of fake Galil
        @param[in] wakeUpHomed: should actuators be homed upon construction, or not?
        @param[in] stateCallback: function to call when connection state of hardware controller or device changes;
            receives one argument: this device wrapper
        """
        self._mirror = mirror
        self.readyDeferred = Deferred()
        self._closeDeferred = None
        self._stateCallback = stateCallback
        self._isReady = False
        self.device = None # the Fake GalilDevice, once it's built
        
        self.hardwareController = galilClass(
            mirror=mirror,
            port=0,
            verbose=False,
            wakeUpHomed=wakeUpHomed,
            stateCallback=self._hardwareStateChanged,
        )
    
    def _devConnCallback(self, conn):
        """Called when the device's connection state changes
        """
        if not self.readyDeferred.called:
            if conn.isConnected:
                self.readyDeferred.callback("")
            elif conn.didFail:
                self.readyDeferred.errback("")
    
    def _makeDevice(self):
        port = self.hardwareController.port
        print "_makeDevice, port=", port
        self.device = GalilDevice(
            mirror=self._mirror,
            host="localhost",
            port=port,
        )
        self.device.conn.addStateCallback(self._stateChanged)
        self.device.connect()
    
    @property
    def isReady(self):
        """Return True if the fake hardare controller and device are running
        """
        return self._isReady

# this may also be useful, but I'd rather poll the device directly        
#     @property
#     def isConnected(self):
#         """Return True if the device is connected to the fake hardware controller
#         """
#         return self.hardwareController.isReady and self.device.conn.isConnected
    
    @property
    def isDone(self):
        """Return True if the device and fake hardware controller are fully disconnected
        """
        return self.hardwareController.isDone and self.device.conn.isDone
    
    @property
    def didFail(self):
        """Return True if connection or disconnection
        """
        return self.isDone and (self.hardwareController.didFail or self.device.conn.didFail)
    
    def _hardwareStateChanged(self, sock):
        """Called when the fake hardware controller's server socket changes state
        """
        if sock.isReady and not self.device:
            self._makeDevice()
        self._stateChanged()
    
    def _stateChanged(self, *args):
        """Called when state changes
        """
        #print "_stateChanged; self.hardwareController.isReady=%s, self.device.conn.state=%s" % (self.hardwareController.isReady, self.device.conn.state)
        if self._closeDeferred: # closing or closed
            if self.isDone:
                self._isReady = False
                if not self.readyDeferred.called:
                    self.readyDeferred.cancel()
                if not self._closeDeferred.called:
                    self._closeDeferred.callback(None)
                    self._stateCallback = None
                else:
                    sys.stderr.write("Device wrapper state changed after wrapper closed\n")
        else: # opening or open
            if self.hardwareController.isReady and self.device.conn.isConnected:
                self._isReady = True
                if not self.readyDeferred.called:
                    self.readyDeferred.callback(None)
            elif self.didFail and not self.readyDeferred.called:
                self.readyDeferred.errback("Failed") # this should probably be a Twisted error object

        if self._stateCallback:
            safeCall(self._stateCallback, self)
    
    def close(self):
        """Close everything
        
        @return a deferred
        """
        if self._closeDeferred:
            raise RuntimeError("Already closing or closed")

        self._closeDeferred = Deferred()
        if not self.readyDeferred.called:
            self.readyDeferred.cancel()
        self.device.disconnect()
        self.hardwareController.close()
        return self._closeDeferred
