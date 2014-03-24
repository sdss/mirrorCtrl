from __future__ import division, absolute_import
"""Fake Galil actor wrapper.
"""
from twistedActor import DispatcherWrapper

from .fakeGalil import FakeGalil
from .fakeMirrorCtrlWrapper import FakeMirrorCtrlWrapper

__all__ = ["FakeDispatcherWrapper"]

class FakeDispatcherWrapper(DispatcherWrapper):
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
        debug = False,
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
        @param[in] debug: print debug messages to stdout?
        """
        actorWrapper = FakeMirrorCtrlWrapper(
            mirror=mirror,
            galilClass=galilClass,
            userPort=userPort,
            verbose=verbose,
            wakeUpHomed=wakeUpHomed,
            debug=debug,
        )
        DispatcherWrapper.__init__(self,
            dictName=dictName,
            actorWrapper=actorWrapper,
            readCallback=readCallback,
            stateCallback=stateCallback,
            debug=debug,
        )
        self._mirror = mirror
        self.dispatcher = None # the ActorDispatcher, once it's built
