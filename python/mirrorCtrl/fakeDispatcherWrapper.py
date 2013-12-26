#!/usr/bin/env python
"""Fake Galil actor wrapper.
"""
import sys

import RO.Comm.Generic
RO.Comm.Generic.setFramework("twisted")
from opscore.actor import ActorDispatcher
from RO.AddCallback import safeCall
from RO.Comm.TCPConnection import TCPConnection
from twisted.internet.defer import Deferred

from .fakeGalil import FakeGalil
from .fakeMirrorCtrlWrapper import FakeMirrorCtrlWrapper

__all__ = ["FakeDispatcherWrapper"]

class FakeDispatcherWrapper(object):
    """A wrapper for an ActorDispatcher talking to a mirror controller talking to a fake Galil
    
    This wrapper is responsible for starting and stopping everything:
    - It builds a FakeMirrorCtrlWrapper on construction
    - It builds an ActorDispatcher when fake mirror controller is ready
    - It stops both on close()
    
    Public attributes include:
    - actorWrapper: the mirror controller actor (the name reflects the generic case)
    - dispatcher: the ActorDispatcher (None until ready)
    - readyDeferred: called when the dispatcher is ready
      (for tracking closure use the Deferred returned by the close method, or stateCallback).
    """
    def __init__(self,
        mirror,
        userPort = 0,
        dictName = "mirror",
        galilClass = FakeGalil,
        verbose = False,
        wakeUpHomed = True,
        readCallback = None,
        stateCallback = None,
    ):
        """Construct a FakeDispatcherWrapper that manages everything

        @param[in] userPort: port for mirror controller connections; 0 to auto-select
        @param[in] dictName: name of actor key dictionary
        @param[in] mirror: the Mirror object used by the fake Galil
        @param[in] galilClass: class of fake Galil
        @param[in] verbose: should the fake Galil run in verbose mode?
        @param[in] wakeUpHomed: should actuators be homed upon construction, or not?
        @param[in] readCallback: function to call when the dispatcher has data to read
        @param[in] stateCallback: function to call when connection state of hardware controller or actor changes;
            receives one argument: this actor wrapper
        """
        self._mirror = mirror
        self._dictName = dictName
        self.readyDeferred = Deferred()
        self._closeDeferred = None
        self._readCallback = readCallback
        self._stateCallback = stateCallback
        self.dispatcher = None # the ActorDispatcher, once it's built
        
        self.actorWrapper = FakeMirrorCtrlWrapper(
            mirror=mirror,
            galilClass=galilClass,
            userPort=userPort,
            verbose=verbose,
            wakeUpHomed=wakeUpHomed,
            stateCallback=self._actorWrapperStateChanged,
        )
    
    def _actorConnCallback(self, conn):
        """Called when the actor's connection state changes
        """
        if not self.readyDeferred.called:
            if conn.isReady:
                self.readyDeferred.callback("")
            elif conn.didFail:
                self.readyDeferred.errback("")
    
    def _makeDispatcher(self):
        #print "_makeDispatcher()"
        connection = TCPConnection(
            host = 'localhost',
            port = self.actorWrapper.userPort,
            readCallback = self._readCallback,
            stateCallback = self._stateChanged,
            readLines = True,
            name = "mirrorCtrlConn",
        )
        self.dispatcher = ActorDispatcher(
            connection = connection,
            name = self._dictName, # name of keyword dictionary
        )
        connection.connect()
    
    @property
    def actor(self):
        """Return the actor (in this case, the mirror controller)
        """
        return self.actorWrapper.actor
    
    @property
    def userPort(self):
        """Return the actor port, if known, else None
        """
        return self.actorWrapper.userPort
        
    @property
    def isReady(self):
        """Return True if the actor has connected to the fake hardware controller
        """
        return self.actorWrapper.isReady and self.dispatcher and self.dispatcher.connection.isConnected
    
    @property
    def isDone(self):
        """Return True if the actor and fake hardware controller are fully disconnected
        """
        return self.actorWrapper.isDone and self.dispatcher and self.dispatcher.connection.isDisconnected
    
    @property
    def didFail(self):
        """Return True if isDone and there was a failure
        """
        return self.isDone and (self.actorWrapper.didFail or self.dispatcher.connection.didFail)
    
    def _actorWrapperStateChanged(self, devWrapper):
        """Called when the device wrapper changes state
        """
        if devWrapper.isReady and not self.dispatcher:
            self._makeDispatcher()
        self._stateChanged()
    
    def _stateChanged(self, *args):
        """Called when state changes
        """
#         print "%r; _stateChanged; self.actorWrapper.isReady=%s, self.actorWrapper.isDone=%s, self.dispatcher.connection.state=%s, self.closeDeferred.called=%s" % \
#             (self, self.actorWrapper.isReady, self.actorWrapper.isDone, self.dispatcher.connection.state if self.dispatcher else "?",
#             self._closeDeferred.called if self._closeDeferred else "?")
        if self._closeDeferred: # closing or closed
            if self.isDone:
                if not self.readyDeferred.called:
                    self.readyDeferred.cancel()
                if not self._closeDeferred.called:
#                    print "\n*** %s calling closeDeferred ***\n" % (self,)
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
        if self.dispatcher:
            self.dispatcher.disconnect()
        self.actorWrapper.close()
        return self._closeDeferred
    
    def __str__(self):
        return "%s" % (type(self).__name__,)
    
    def __repr__(self):
        return "%s; isReady=%s, isDone=%s, didFail=%s" % \
            (type(self).__name__, self.isReady, self.isDone, self.didFail)
