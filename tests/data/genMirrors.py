"""This script generates actuator/encoder lists, and enters them into seperate 
mirror objects.  This mirrors are used for unit testing purposes."""

import math
import itertools

import numpy
import matplotlib.pyplot

import mirrorCtrl

RadPerDeg = math.pi / 180.0
    
class ConstMirrorBase(object):
    """This object is used to construct APO mirrors to be used in unit testing
    """
    def plotMirror(self):
        """ Plots links and glass when the mirror is in neutral position"""
        fig = matplotlib.pyplot.figure()
        ax = fig.gca(projection='3d')
        theta = numpy.linspace(0., 2 * numpy.pi, 100)
        z = numpy.zeros(100)
        fig.hold()
        
        # plot the mirror black concentric circles
        for r in numpy.linspace(0., self.mirRad, 50):
            x = r * numpy.sin(theta)
            y = r * numpy.cos(theta)
            if self.mirRad < 300: # mm
                # 3.5m M3 mirror is rotated about x by -45 deg
                phiRad = 90.0 * RadPerDeg
                cosP = math.cos(phiRad)
                sinP = math.sin(phiRad)
                stack = numpy.vstack((x,y,z))
                stack = stack.T
                rotMat = numpy.array([ [1., 0., 0.],
                                       [0., cosP, sinP],
                                       [0., -sinP, cosP] ])
                stackRot = numpy.dot(stack, rotMat)
                x = stackRot[:,0]
                y = stackRot[:,1]
                z = stackRot[:,2]
            ax.plot(x, y, z, 'k')
        
        # plot actuators blue
        for act in self.actList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])  
            y = numpy.array([act.mirPos[1], act.basePos[1]]) 
            z = numpy.array([act.mirPos[2], act.basePos[2]])   
            ax.plot(x, y, z, 'bo-')
        
        # plot encoders cyan
        for act in self.encList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])  
            y = numpy.array([act.mirPos[1], act.basePos[1]]) 
            z = numpy.array([act.mirPos[2], act.basePos[2]])   
            ax.plot(x, y, z, 'co-')
        
        # plot fixed links red
        for act in self.fixList:
            x = numpy.array([act.mirPos[0], act.basePos[0]])  
            y = numpy.array([act.mirPos[1], act.basePos[1]]) 
            z = numpy.array([act.mirPos[2], act.basePos[2]])   
            ax.plot(x, y, z, 'ro-')
        
        xyRange=(-1.5 * self.mirRad, 1.5 * self.mirRad)
        # zRange = (-3 * self.mirRad, 2 * self.mirRad)
        matplotlib.pyplot.xlim(xyRange)
        matplotlib.pyplot.ylim(xyRange)
        ax.set_zlim(xyRange)        
        matplotlib.pyplot.show()
        
    def genActuators(self, minMnt, maxMnt, mntOffset, mntScale, mirPos, basePos, actType):
        """Generate a list of actuators, to be used in a mirror.   
        
        Inputs (n = number of actuators supplied):
        
        min[n]: mount min 
        max[n]: mount max 
        offset[n]: mount offset
        scale[n]: mount scale
        mirPos[n,3]: cartesian mirror position (rows of xyz!)
        basePos[n,3]: cartesian base position (rows of xyz!)
        actType: may be either 'adjBase' or 'adjLen'.    
                 
        Output:
        actList[n]: list of n actuator objects.
        """    
        if actType == 'adjLen':
            genLink = mirrorCtrl.AdjLengthLink
        elif actType == 'adjBase':
            genLink = mirrorCtrl.AdjBaseActuator
        else:
            # earlier checks should make getting here impossible, but....
            raise RuntimeError('actType must be either "adjLen" or "adjBase"')
        actList = [ genLink(base, mir, min, max, scale, off) 
                    for base, mir, min, max, scale, off 
                    in itertools.izip(basePos, mirPos, minMnt, maxMnt,
                                      mntScale, mntOffset) ]
                                    
        return actList
        
    def genFakeEncPos(self, mirPos, basePos, rad=75, vert=75):
        """Generate a list of fake encoders, to be used in a mirror.
    
        Encoders are type: AdjLengthLinks (not AdjBaseActuators). Fake
        encoder positions are estimated by offseting the nominal (input)
        actuator positions in the following manner:
    
        Piston type actuators are offset radially (in the xy plane) 75 mm. 
        First 3 supplied actuators are assumed to be pistions.
    
        All other actuators are assumed to be transverse.  These are offset
        vertically (+ Z) by 75 mm
    
    
        Inputs (n = number of actuators supplied):
    
        min[n]: mount min 
        max[n]: mount max 
        offset[n]: mount offset
        scale[n]: mount scale
        mirPos[n,3]: cartesian mirror position (rows of xyz!)
        basePos[n,3]: cartesian base position (rows of xyz!)
    
        Output: 
        adjMirPos[n,3]: new mirPos
        adjBasePos[n,3]: new basePos"""
        
        adjMirPos = mirPos[:]
        adjBasePos = basePos[:]
        
        # offset in xy radially (away from center) for pistons
        x = mirPos[0:3,0]
        y = mirPos[0:3,1]
        # radial length
        mag = numpy.sqrt(x**2. + y**2.)
        xNew = x * (mag + rad) / mag
        yNew = y * (mag + rad) / mag
        # z position unaffected
        # plug in new x and y for first 3 (just piston) actuator positions
        adjMirPos[0:3,0:2] = numpy.vstack((xNew, yNew)).T  # .T for xy in 2nd dim
        adjBasePos[0:3,0:2] = numpy.vstack((xNew, yNew)).T
        # Now vertical offsets for transverse encoders.
        if len(mirPos) > 3:
            adjMirPos[3:,2] += vert
            adjBasePos[3:,2] += vert
        return adjMirPos, adjBasePos
        
    def genFixedLinks(self, basePos, mirPos):
        """Generate a list of fixed links, to be used in a mirror.
        
        Inputs (n = number of fixed links supplied):
        
        basePos[n,3]: cartesian base position (rows of xyz!)
        mirPos[n,3]: cartesian mirror position
        
        Output: fixedList[n]: list of fixed link objects"""
        
        # make sure that mirror/base positions have xyz along 2nd axis
        if basePos.shape == (3,):
            basePos = numpy.array([basePos])
            mirPos = numpy.array([mirPos])
        if basePos.shape[1] != 3 or mirPos.shape[1] != 3:
            raise RuntimeError('mir/base positions must have xyz coords along 2nd axis!')
        
        fixedList = [mirrorCtrl.FixedLengthLink(base, mir) for base, mir in itertools.izip(basePos, mirPos)]
        return fixedList
        
        
class ConstDirectMirror(ConstMirrorBase):
    """Direct Mirror Constructor"""
        
    def makeMirror(self, name=None):
        """Returns a Direct mirror ready for use"""
        return mirrorCtrl.DirectMirror(self.actList, self.fixList, self.encList, name) #must specify mirror id....
        
        
class ConstTipTransMirror(ConstMirrorBase):
    """TipTrans Mirror Constructor"""
        
    def makeMirror(self, name=None):
        """Returns a TipTrans mirror ready for use.
        
        Ctr base/mir pos are hard-coded for the 2.5m secondary, since that is the only 
        tip trans mirror we have."""
    
        secCtrMirZ = -135.70
        secCtrBaseZ = -178.40
        return mirrorCtrl.TipTransMirror(secCtrMirZ, secCtrBaseZ, self.actList, 
                                     self.fixList, self.encList, name)
                                     

class Prim25(ConstDirectMirror):
    """The SDSS 2.5m Primary Mirror"""
    def __init__(self, fix=None, vers='old', actType='adjBase'):
        """create the usable mirror.
        
        If fix is specified as 1 or 2, a tranverse actuator is taken to be fixed (emulating
        a fixed link version of this mirror. 1 fixes the +Y transverse X actuator, 
        2 fixes -Y transverse X actuator. Default is None, where no actuators are held fixed.
        
        If vers = 'new', the transverse Y actuator is attached to mirror vertex.
        If vers = 'old', the transerse Y actuator is attached to edge of mirror at -Y. 
                     This is default.
               
        Inputs:
        fix: None, 1, or 2.
        vers: 'old' or 'new'
        actType: 'adjBase' or 'adjLen', which type of actuator to use.
        """
        if (fix not in [None, 1, 2]):
            raise RuntimeError('Fix must be None, 1, or 2.')
        if (vers not in ['old', 'new']):
            raise RuntimeError('vers must be "old" or "new"')
        if (actType not in ['adjBase', 'adjLen']):
            raise RuntimeError('actType must be "adjBase" or "adjLen"')
        
        self.mirRad = 1250. # mm
        self.actList, self.fixList, self.encList = self.dataToLists(fix, vers, actType)
        
    def dataToLists(self, fix, vers, actType):
        """Returns actuator, fixedLink, encoder lists for this mirror. FixedLink list is empty.
        
        Actuator position data supplied by 25m_mir.dat, hard-coded below.  Infinite values
        9e9 were adjusted to 9e4 to avoid round off errors"""
        
        min = numpy.array([-120000., -120000., -120000., -90000., -50000., -50000])
        max = numpy.array([ 120000.,  120000.,  120000.,  90000.,  50000.,  50000])
        
        offset = numpy.array([11300.,  -650.,  5500., -1650., -6900., -6900])
        # scale  = numpy.array([15.696, 15.696, 15.696, 15.696,  33.22, 32.53])  #<--True scale
        # choose a higher resolution scale, same as 2.5m M2
        scale  = numpy.array([1259.84, 1259.84, 1259.84, 31.496, 31.496, 31.496])
        
        mirX  = numpy.array([    0., -749.03,  749.03,     0.,     0.,    0.])
        mirY  = numpy.array([864.90, -432.45, -432.45, -1305., -1277., 1277.])
        mirZ  = numpy.array([  251.,    251.,    251.,   238.,   262.,  262.])
        # modified: attach trans y act to vertex
        if vers == 'new':
            mirY  = numpy.array([864.90, -432.45, -432.45,      0., -1277., 1277.]) 
                
        #----------- these are the original values:
        # baseX = numpy.array([    0., -749.03,  749.03,     0.,  -698., -698.])
        # baseY = numpy.array([864.90, -432.45, -432.45,   -9e9, -1277., 1277.])
        # baseZ = numpy.array([   9e9,     9e9,     9e9,   238.,   262.,  262.])
               
        # the solver in mirror.py works by computing length differences, 
        # so big numbers could be prone to large machine round-off errors.
        
        # A more finite version (replaced 9e9 with 9e4):
        baseX = numpy.array([    0., -749.03,  749.03,     0.,  -698., -698.])
        baseY = numpy.array([864.90, -432.45, -432.45,   -9e4, -1277., 1277.])
        baseZ = numpy.array([   9e4,     9e4,     9e4,   238.,   262.,  262.])
        
        # xyz must be along 2nd dimension, hence the transpose
        mirPos  = numpy.vstack((mirX, mirY, mirZ)).T
        basePos = numpy.vstack((baseX, baseY, baseZ)).T
        
        if fix == None:
            # hold no actuators constant
            actList = self.genActuators(min, max, offset, scale, mirPos, basePos, actType)
            fixList = []
            encMirPos, encBasePos = self.genFakeEncPos(mirPos, basePos)
            # encoders are type AdjLengthLink.
            encList = self.genActuators(min, max, offset, scale, encMirPos, 
                                        encBasePos, 'adjLen')
        elif fix == 1:
            # hold actuator #6 constant
            actList = self.genActuators(min[0:5], max[0:5], offset[0:5], 
                                        scale[0:5], mirPos[0:5,:], basePos[0:5,:], actType)
            fixList = self.genFixedLinks(basePos[5,:], mirPos[5,:])
            encMirPos, encBasePos = self.genFakeEncPos(mirPos[0:5,:], basePos[0:5,:])
            # encoders are type AdjLengthLink.
            encList = self.genActuators(min[0:5], max[0:5], offset[0:5], scale[0:5], encMirPos, 
                                        encBasePos, 'adjLen')   
        elif fix == 2:
            # hold actuator #5 constant
            index = [0, 1, 2, 3, 5]
            actList = self.genActuators(min[index], max[index], offset[index], 
                                        scale[index], mirPos[index,:], basePos[index,:], actType)
            fixList = self.genFixedLinks(basePos[4,:], mirPos[4,:])
            encMirPos, encBasePos = self.genFakeEncPos(mirPos[index,:], basePos[index,:])
            # encoders are type AdjLengthLink.
            encList = self.genActuators(min[index], max[index], offset[index], scale[index], encMirPos, 
                                        encBasePos, 'adjLen')   
        return actList, fixList, encList
        
class Sec25(ConstTipTransMirror):
    """The SDSS 2.5m Secondary Mirror"""
    def __init__(self, actType='adjBase'):
        """create the usable mirror"""
        if (actType not in ['adjBase', 'adjLen']):
            raise RuntimeError('actType must be "adjBase" or "adjLen"')
        self.mirRad = 417. # mm (estimated)
        self.actList, self.fixList, self.encList = self.dataToLists(actType)
        
    def dataToLists(self, actType):
        """Returns actuator, fixedLink, encoder lists for this mirror. FixedLink is an anti
        z rotation link. Its location was dreamed up (hopefully reasonably) by me. Otherwise
        actuator position data supplied by 25m_mir.dat, hard-coded below."""
        
        min = numpy.array([-7250000., -7250000., -7250000., -18000., -18000])
        max = numpy.array([ 7250000.,  7250000.,  7250000.,  18000.,  18000])
        
        offset = numpy.array([     0.,      0.,      0.,  1700., -1700.])
        scale  = numpy.array([1259.84, 1259.84, 1259.84, 31.496, 31.496])
        
        mirX  = numpy.array([ 293.81, -233.08,  -60.73,   19.80,  -19.80])
        mirY  = numpy.array([  99.51,  204.69, -304.20,  -19.80,  -19.80])
        mirZ  = numpy.array([-193.00, -193.00, -193.00, -263.80, -263.80])
        baseX = numpy.array([ 293.81, -233.08,  -60.73,   56.57,  -56.57])
        baseY = numpy.array([  99.51,  204.69, -304.20,  -56.57,  -56.57])
        baseZ = numpy.array([-280.00, -280.00, -280.00, -263.80, -263.80])
        
        # xyz must be along 2nd dimension, hence the transpose
        mirPos  = numpy.vstack((mirX, mirY, mirZ)).T
        basePos = numpy.vstack((baseX, baseY, baseZ)).T
        
        # create a fake FixedLengthLink to constrain z rotation, extending in x, length = 150 mm
        linkLength = 150. #mm
        mirRadius = self.mirRad
        fixMirPos = numpy.array([[0., mirRadius, -193.]])
        fixBasePos = numpy.array([[linkLength, mirRadius, -193.]])
        
        actList = self.genActuators(min, max, offset, scale, mirPos, basePos, actType)
        fixList = self.genFixedLinks(fixBasePos, fixMirPos)
        encMirPos, encBasePos = self.genFakeEncPos(mirPos, basePos)
        # encoders are type AdjLengthLink.
        encList = self.genActuators(min, max, offset, scale, encMirPos, 
                                    encBasePos, 'adjLen')
        
        return actList, fixList, encList
        
class Sec35(ConstDirectMirror):
    """The ARC 3.5m secondary mirror"""
    def __init__(self, actType='adjBase'):
        """create the usable mirror"""
        if (actType not in ['adjBase', 'adjLen']):
            raise RuntimeError('actType must be "adjBase" or "adjLen"')
        self.mirRad = 417. # mm (true size, from Nick MacDonald solid model)
        self.actList, self.fixList, self.encList = self.dataToLists(actType)
        
    def dataToLists(self, actType):
        """Returns actuator, fixedLink, encoder lists for this mirror. FixedLink is an anti
        z rotation link. Its location was dreamed up (hopefully reasonably) by me. Otherwise
        actuator position data supplied by mir_35m.dat, hard-coded below."""
        
        min = numpy.array([-7250000., -7250000., -7250000., -95000., -95000])
        max = numpy.array([ 7250000.,  7250000.,  7250000.,  95000.,  95000])
        
        offset = numpy.array([      0.,       0.,       0.,     0.,     0.])
        scale  = numpy.array([1259.843, 1259.843, 1259.843, 31.496, 31.496])
        
        mirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
        mirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
        mirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
        baseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
        baseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
        baseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])        
        
        # Fake FixedLengthLink
        linkLength = 150. #mm
        mirRadius = self.mirRad
        fixMirPos = numpy.array([[0., mirRadius, -152.806]])
        fixBasePos = numpy.array([[linkLength, mirRadius, -152.806]])
        
        # xyz must be along 2nd dimension, hence the transpose
        mirPos  = numpy.vstack((mirX, mirY, mirZ)).T
        basePos = numpy.vstack((baseX, baseY, baseZ)).T
        
        actList = self.genActuators(min, max, offset, scale, mirPos, basePos, actType)
        fixList = self.genFixedLinks(fixBasePos, fixMirPos)
        encMirPos, encBasePos = self.genFakeEncPos(mirPos, basePos)
        # encoders are type AdjLengthLink.
        encList = self.genActuators(min, max, offset, scale, encMirPos, 
                                    encBasePos, 'adjLen')
        
        return actList, fixList, encList        
        
class Tert35(ConstDirectMirror):
    """The ARC 3.5m tertiary mirror"""
    def __init__(self, vers='new', actType='adjBase'):
        """create the usable mirror"""
        if (vers not in ['old', 'new']):
            raise RuntimeError('vers must be "old" or "new"')
        if (actType not in ['adjBase', 'adjLen']):
            raise RuntimeError('actType must be "adjBase" or "adjLen"')
        
        self.mirRad = 297.54 # mm ( derived from 3.5m M3 Actuator Positions 2008-04-18.py)
        self.actList, self.fixList, self.encList = self.dataToLists(vers, actType)
        
    def dataToLists(self, vers, actType):
        """Returns actuator, fixedLink, encoder lists for this mirror. 
        
        The 3.5m tertiary has a different coordinate system convention:
        +Z points towards secondary
        +Y points towards instrument port
        +X defined by right hand rule
        
        The actuator placements are defined (below) in the plane of the mirror, so a
        coord transform is necessary.  The coords and transform method are almost identical 
        to the file: 3.5m M3 Actuator Positions 2008-04-18.py (in docs directory).  
        Differences are:
        1. Fake encoder positions are also computed here.
        2. Fixed link geometry is from Nick Macdonald, the old version used 3
           infinitely long links fixed at different positions than they are in reality.
           This version uses short rods, attached at their true physical locations (although
           the X translation link wasn't in the solid model, so I guessed that it extends along
           +X, rather than -X).
           The old positions are included below for reference but commented out."""
        
        # -------BEGIN copying from 3.5m M3 Actuator Positions 2008-04-18.py--------
        MMPerInch = 25.4
        RadPerDeg = math.pi / 180.0

        rad =   11.714 * MMPerInch
        zMir =  -0.875 * MMPerInch
        zBase = -3.375 * MMPerInch
        angDegList = numpy.arange(-90.0, 359.0, 360.0 / 3.0)
        angRadList = angDegList * RadPerDeg
        
        # compute in-plane positions
        # with convention:
        # z points from back of glass to front of glass
        # y points sort of towards the instrument
        # x is as required for right-handed coordinate system
        # Actuator index:
        #   0-2 = axial actuators A-C
        #   3-5 = constraints
        mirIP = numpy.zeros([6,3])
        baseIP = mirIP.copy()
        # first handle actuators A, B and C
        # could compute encoders here too by adjusting rad, but I do it below
        for actInd, angRad in enumerate(angRadList):
            mirIP[actInd, :] = numpy.array((
                math.cos(angRad) * rad,
                math.sin(angRad) * rad,
                zMir
            ))
            baseIP[actInd, 0:2] = mirIP[actInd, 0:2]
            baseIP[actInd, 2] = zBase
            
        # now add constraints:
        # 3,4 are transverse
        # 5 is anti-rotation
        
        if vers == 'old':
            # ------- old implementation ----------
            # infinite links below are taken to be 1e4 to avoid
            # roundoff error in new implementation (old version used 1e9)
            mirIP[3] = numpy.zeros(3)
            mirIP[4] = mirIP[3].copy()
            mirIP[5] = mirIP[3].copy()
            mirIP[5,0] = rad
            baseIP[3] = mirIP[3].copy()
            baseIP[3, 0:2] = (1.0e4, 1.0e4)   
            baseIP[4, 0:2] = (-1.0e4, 1.0e4)
            baseIP[5] = (rad, 1.0e4, 0)
    
        if vers == 'new':   
            # actual positions (from Nick MacDonald)
            mirIP[3] = (-203.2, 0., 0.)
            mirIP[4] = (203.2, 0., 0.)
            mirIP[5] = (0., 0., 0.)
            baseIP[3] = mirIP[3].copy()
            baseIP[3, 1] = (281.47)
            baseIP[4] = mirIP[4].copy()
            baseIP[4, 1] = (281.47) 
            baseIP[5] = (281.47, 0., 0.)
        
        # make encoder positions before transforming coords!
        encMirIP, encBaseIP = self.genFakeEncPos(mirIP[0:3,:], baseIP[0:3,:])

        
        # rotate to final coordinate system which is:
        # z points towards secondary
        # y points towards the instrument port
        # x is unchanged
        # in other words, rotate 45 degrees about x
        rotAng = -45.0 * math.pi / 180.0
        rotMat = numpy.zeros([3,3])
        rotMat[0,0] = 1
        rotMat[1,1] = math.cos(rotAng)
        rotMat[1,2] = math.sin(rotAng)
        rotMat[2,2] = rotMat[1,1]
        rotMat[2,1] = -rotMat[1,2]
        mirPos = numpy.dot(mirIP, rotMat)
        basePos = numpy.dot(baseIP, rotMat)
        encMirPos = numpy.dot(encMirIP, rotMat)
        encBasePos = numpy.dot(encBaseIP, rotMat)
        
        # -------END (mostly) copying from 3.5m M3 Actuator Positions 2008-04-18.py--------
        
        # xyz must be along 2nd dimension, hence the transpose
        
        # from mir_35.dat:
        min = numpy.array([-7250000., -7250000., -7250000])
        max = numpy.array([ 7250000.,  7250000.,  7250000])
        offset = numpy.array([      0.,       0.,       0.])
        scale  = numpy.array([1259.843, 1259.843, 1259.843])

        # first 3 are actuators
        actList = self.genActuators(min, max, offset, scale, mirPos[0:3,:], basePos[0:3,:], actType)
        # last 3 are fixed links
        fixList = self.genFixedLinks(basePos[3:,:], mirPos[3:,:])
        encList = self.genActuators(min, max, offset, scale, encMirPos, encBasePos, 'adjLen')
        
        return actList, fixList, encList
