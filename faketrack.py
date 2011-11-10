#!/usr/bin/env python
"""Script to send regular MOVE P V T commands
"""
import sys
import time
import Tkinter
import numpy
import RO.Comm.TCPConnection
import RO.CoordSys
import RO.TkUtil
import RO.Astro.Cnv
import RO.Astro.Sph
import RO.Astro.Tm
import RO.MathUtil
import RO.Wdg

TrackInterval = 0.2 # time between outputs (sec)
AdvTime = 0.0 # time in advance that outputs are sent (sec)
AzAddr = None
AzPort = None
AltAddr = None
AltPort = None

Longitude = -105.820417 # longitude east, deg
Latitude  =   32.780361 # latitude north, deg
Elevation =    2.788    # elevation, km

ObsData = RO.Astro.Cnv.ObserverData(
    longitude = Longitude,
    latitude = Latitude,
    elevation = Elevation,
)

class ComputePosVel(object):
    _velDT = 0.1 # seconds
    def __init__(self, obsData):
        """Create a ComputePosVel
        
        Inputs:
        - obsData: instance of RO.Astro.Cnv.ObserverData for your telescope
        """
        self.obsData = obsData
        self.fromSys = "ICRS"
        self.fromDate = 2000.0
        self.toSys = "Topocentric"
        self.fromPos = None
        self.fromVel = numpy.array((0.0, 0.0))
        self.startTime = time.time()
    
    def setTarget(self, pos, vel=None, csys=None, date=None):
        if len(pos) != 2:
            raise RuntimeError("pos must be a sequence of length 2")
        pos = numpy.array(pos, dtype=float)
        if vel == None:
            vel = (0, 0)
        elif len(vel) != 2:
            raise RuntimeError("vel=%s; must be a sequence of length 2")
        vel = numpy.array(vel, dtype=float)
        if csys != None:
            self.fromSys = RO.CoordSys.getSysConst(csys)
            self.fromDate = date
        self.fromPos = pos
        self.fromVel = vel
        self.startTime = time.time()
        self.prevToPos = None

    def computeMount(self):
        """Return mount position, velocity and TAI
        
        Mount is really Apparent Topocentric with unwrap, but oh well.
        """
        if self.fromPos == None:
            raise runtimeError("No position!")
        startUnixTime = time.time() + AdvTime
        cnvPosArr = []
        ut1Arr = []
        for unixTime in (startUnixTime, startUnixTime + self._velDT):
            ut1 = RO.Astro.Tm.utcFromPySec(unixTime)
            dUnixTime = unixTime - self.startTime
            pos = self.fromPos + (dUnixTime * self.fromVel)
            cnvPos, toPM, toParlax, toRadVel, toDir, scaleChange, atInf, atPole = \
                RO.Astro.Sph.coordConv(
                    fromPos = pos,
                    fromSys = self.fromSys,
                    fromDate = None,
                    toSys = self.toSys,
                    toDate = ut1,
                    obsData = self.obsData,
                )
            cnvPosArr.append(numpy.array(cnvPos))
            ut1Arr.append(ut1)
        toPos = cnvPosArr[0]
        
        # unwrap toPos[0]
        if self.prevToPos != None:
            azDiff = RO.MathUtil.wrapCtr(toPos[0] - self.prevToPos[0])
            toPos[0] = self.prevToPos[0] + azDiff
        self.prevToPos = toPos
        toVel = (cnvPosArr[1] - cnvPosArr[0]) / self._velDT
        tai = RO.Astro.Tm.taiFromPySec(startUnixTime)

        return toPos, toVel, tai

class ConnWdg(Tkinter.Frame):
    def __init__(self, master, addr, port, width=50):
        Tkinter.Frame.__init__(self, master)
        self.conn = RO.Comm.TCPConnection.TCPConnection(
            host = addr,
            port = port,
            stateCallback = self.stateCallFunc,
            readCallback = self.readCallFunc,
        )
        self.logWdg = RO.Wdg.LogWdg(
            master = self,
            width = width,
        )
        self.logWdg.grid(row=0, column=0)
        self.cmdWdg = RO.Wdg.CmdWdg(
            master = self,
            cmdFunc = self.cmdFunc,
        )
        self.cmdWdg.grid(row=1, column=0, sticky="ew")

    def stateCallFunc(self, dumArg=None):
        """State callback from connection
        """
        state, stateStr, reason = self.conn.getState()
        self.logWdg.addMsg("Connection state: %s %s" % (stateStr, reason))
    
    def readCallFunc(self, dumArg=None):
        """Read callback from connection
        """
        msgStr = self.conn.readLine()
        self.logWdg.addMsg(msgStr)
    
    def cmdFunc(self, cmdStr):
        """Callback from command bar
        """
        self.writeLine(cmdStr)
    
    def writeLine(self, msgStr):
        """Write a line to the controller and log it
        
        If disconnected then log it in parenthesis
        """
        if self.conn.isConnected():
            self.conn.writeLine(msgStr)
            self.logWdg.addMsg("Sent: " + msgStr)
        else:
            self.logWdg.addMsg("Want: " + msgStr)


class FakeTrackWdg(Tkinter.Frame):
    def __init__(self, master):
        self.pvComputer = ComputePosVel(obsData = ObsData)
        Tkinter.Frame.__init__(self, master)
        self.trackTimer = RO.TkUtil.Timer()
        gr = RO.Wdg.Gridder(self)

        self.pos1Wdg = RO.Wdg.FloatEntry(
            master = self,
        )
        self.pos2Wdg = RO.Wdg.FloatEntry(
            master = self,
        )
        gr.gridWdg("RA, Dec Pos: ", (self.pos1Wdg, self.pos2Wdg), "deg")
        self.vel1Wdg = RO.Wdg.FloatEntry(
            master = self,
        )
        self.vel2Wdg = RO.Wdg.FloatEntry(
            master = self,
        )
        gr.gridWdg("RA, Dec Vel: ", (self.vel1Wdg, self.vel2Wdg), "deg/sec")

        logFrame = Tkinter.Frame(self)
        RO.Wdg.StrLabel(logFrame, "Azimuth").grid(row=0, column=0)
        self.azConnWdg = ConnWdg(
            master = logFrame,
            addr = AzAddr,
            port = AzPort,
        )
        self.azConnWdg.grid(row=1, column=0)

        RO.Wdg.StrLabel(logFrame, "Altitude").grid(row=0, column=1)
        self.altConnWdg = ConnWdg(
            master = logFrame,
            addr = AltAddr,
            port = AltPort,
        )
        self.altConnWdg.grid(row=1, column=1)

        gr.gridWdg(False, logFrame, colSpan=6)
        
        btnFrame = Tkinter.Frame(self)
        self.goBtn = RO.Wdg.Button(
            master = btnFrame,
            text = "Go",
            callFunc = self.doGo,
        )
        self.goBtn.pack(side="left")
        self.stopBtn = RO.Wdg.Button(
            master = btnFrame,
            text = "Stop",
            callFunc = self.doStop,
        )
        self.stopBtn.pack(side="left")
        gr.gridWdg(False, btnFrame, colSpan=6, sticky="w")
        
        self.grid_columnconfigure(5, weight=1)

        if AzAddr:
            self.azConnWdg.logWdg.addMsg("AzAddr:AzPort = %s:%s" % (AzAddr, AzPort))
            self.azConnWdg.conn.connect()
        else:
            self.azConnWdg.logWdg.addMsg("No az controller; AzAddr = %s" % (AzAddr,))

        if AltAddr:
            self.altConnWdg.logWdg.addMsg("AltAddr:AltPort = %s:%s" % (AltAddr, AltPort))
            self.altConnWdg.conn.connect()
        else:
            self.altConnWdg.logWdg.addMsg("No alt controller; AltAddr = %s" % (AltAddr,))
        
    def doGo(self, wdg=None):
        pos = self.pos1Wdg.getNum(), self.pos2Wdg.getNum()
        vel = self.vel1Wdg.getNum(), self.vel2Wdg.getNum()
        self.pvComputer.setTarget(pos=pos, vel=vel)
        self.track(True)
    
    def doStop(self, wdg=None):
        self.track(False)
    
    def track(self, doTrack):
        self.trackTimer.cancel()
        if not doTrack:
            self.azConnWdg.writeLine("AZ MOVE")
            self.altConnWdg.writeLine("ALT MOVE")
            return
        
        pos, vel, tai = self.pvComputer.computeMount()
        tai = RO.Astro.Tm.taiFromPySec()
        azCmd  = "AZ  MOVE %0.7f %0.7f %0.3f" % (pos[0], vel[0], tai)
        altCmd = "ALT MOVE %0.7f %0.7f %0.3f" % (pos[1], vel[1], tai)
        self.azConnWdg.writeLine(azCmd)
        self.altConnWdg.writeLine(altCmd)
        
        self.trackTimer.start(TrackInterval, self.track, True)

if __name__ == "__main__":
    root = Tkinter.Tk()
    
    wdg = FakeTrackWdg(master=root)
    wdg.grid(row=0, column=0)

    root.mainloop()
