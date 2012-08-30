#!/usr/bin/env python
"""2.5m secondary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.devices.device25mSec import DeviceInfo

UserPort = 2532

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = DeviceInfo.mirror,
        host = DeviceInfo.host,
        port = DeviceInfo.port,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
