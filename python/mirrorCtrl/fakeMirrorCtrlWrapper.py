#!/usr/bin/env python
"""Fake Galil actor wrapper.
"""
import sys

from twisted.internet.defer import Deferred
from RO.AddCallback import safeCall

from .fakeGalil import FakeGalil
from .fakeGalilDeviceWrapper import FakeGalilDeviceWrapper
from .mirrorCtrl import MirrorCtrl

__all__ = ["FakeMirrorCtrlWrapper"]

class FakeMirrorCtrlWrapper(object):
    """A wrapper for a MirrorCtrl talking to a fake Galil
    
    This wrapper is responsible for starting and stopping a fake Galil and a MirrorCtrl:
    - It builds a FakeGalilDeviceWrapper on construction, using an auto-selected port
    - It builds a MirrorCtrl when the fake Galil is ready
    - It stops both on close()
    
    Public attributes include:
    - deviceWrapper: the fake Galil (the name suggests this could be a general-purpose actor wrapper)
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
    ):
        """Construct a FakeMirrorCtrlWrapper that manages its fake Galil

        @param[in] userPort: port for mirror controller connections; 0 to auto-select
        @param[in] mirror: the Mirror object used by the fake Galil
        @param[in] galilClass: class of fake Galil
        @param[in] verbose: should the fake Galil run in verbose mode?
        @param[in] wakeUpHomed: should actuators be homed upon construction, or not?
        @param[in] stateCallback: function to call when connection state of hardware controller or actor changes;
            receives one argument: this actor wrapper
        """
        self._mirror = mirror
        self._userPort = userPort
        self.readyDeferred = Deferred()
        self._closeDeferred = None
        self._stateCallback = stateCallback
        self.actor = None # the MirrorCtrl, once it's built
        
        self.deviceWrapper = FakeGalilDeviceWrapper(
            mirror=mirror,
            galilClass=galilClass,
            verbose=verbose,
            wakeUpHomed=wakeUpHomed,
            stateCallback=self._deviceWrapperStateChanged,
        )
    
    def _actorConnCallback(self, conn):
        """Called when the actor's connection state changes
        """
        if not self.readyDeferred.called:
            if conn.isReady:
                self.readyDeferred.callback("")
            elif conn.didFail:
                self.readyDeferred.errback("")
    
    def _makeActor(self):
        #print "_makeActor()"
        self.actor = MirrorCtrl(
            device=self.deviceWrapper.device,
            userPort=self._userPort,
        )
        self.actor.server.addStateCallback(self._stateChanged)
    
    @property
    def userPort(self):
        """Return the actor port, if known, else None
        """
        if self.actor:
            return self.actor.server.port
        return None
        
    @property
    def isReady(self):
        """Return True if the actor has connected to the fake hardware controller
        """
        return self.deviceWrapper.isReady and self.actor and self.actor.server.isReady
    
    @property
    def isDone(self):
        """Return True if the actor and fake hardware controller are fully disconnected
        """
        return self.deviceWrapper.isDone and self.actor and self.actor.server.isDone
    
    @property
    def didFail(self):
        """Return True if isDone and there was a failure
        """
        return self.isDone and (self.deviceWrapper.didFail or self.actor.server.didFail)
    
    def _deviceWrapperStateChanged(self, devWrapper):
        """Called when the device wrapper changes state
        """
        if devWrapper.isReady and not self.actor:
            self._makeActor()
        self._stateChanged()
    
    def _stateChanged(self, *args):
        """Called when state changes
        """
#        print "%r; _stateChanged; deviceWrapper.isReady=%s, actor.server.state=%s" % (self, self.deviceWrapper.isReady, self.actor.server.state if self.actor else "?")
        if self._closeDeferred: # closing or closed
            if self.isDone:
                if not self.readyDeferred.called:
                    self.readyDeferred.cancel()
                if not self._closeDeferred.called:
#                    print "*** %s calling closeDeferred ***" % (self,)
                    self._closeDeferred.callback(None)
                else:
                    sys.stderr.write("Device wrapper state changed after wrapper closed\n")
        else: # opening or open
            if not self.readyDeferred.called:
                if self.isReady:
                    self.readyDeferred.callback(None)
                elif self.didFail:
                    self.readyDeferred.errback("Failed") # probably should not be a string?

        if self._stateCallback:
            safeCall(self._stateCallback, self)
            if self.isDone:
                self._stateCallback = None
    
    def close(self):
        """Close everything
        
        @return a deferred
        """
        if self._closeDeferred:
            raise RuntimeError("Already closing or closed")

        self._closeDeferred = Deferred()
        if not self.readyDeferred.called:
            self.readyDeferred.cancel()
        if self.actor:
            self.actor.server.close()
        self.deviceWrapper.close()
        return self._closeDeferred
    
    def __str__(self):
        return "%s" % (type(self).__name__,)
    
    def __repr__(self):
        return "%s; isReady=%s, isDone=%s, didFail=%s" % \
            (type(self).__name__, self.isReady, self.isDone, self.didFail)
