#!/usr/bin/env python
"""2.5m primary mirror controller
"""
import mirrorCtrl
from mirrorCtrl.devices.device25mPrim import DeviceInfo

UserPort = 2531

if __name__ == "__main__":
    device = mirrorCtrl.GalilDevice(
        mirror = DeviceInfo.mirror,
        host = DeviceInfo.host,
        port = DeviceInfo.port,
    )
    mirrorCtrl.runMirrorCtrl(device = device, userPort = UserPort)
