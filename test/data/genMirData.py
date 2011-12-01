"""This script generates a bunch of actuator/encoder/fixed mirror collections into
the MirSetUp object. A list of these mirror objects is generated for each mirror
at APO, and these lists are inteded for unit testing purposes.  The cartesian mirror data
come from 25m_mir.dat and mir_35m.dat.

The lists are:
prim25List[8]: includes 2 faked fixed mirror versions and faked encoders
sec25List[4]: includes a faked fixed mirror and faked encoders
sec35List[4]: includes a faked fixed mirror and faked encoders
tert35List[2]: 

Individual list breakdowns are shown lower in the code, next to each list, but
in general odd indices of each list are constructed using new actuator geometry,
even indices correspond to the old model of an actuator.  

Faked encoders are made by offsetting the supplied
actuator positions by a default distance of 75 mm, radially for piston
actuators, and vertically for transverse actuators.  Faked fixed links are made
by generating a mirror tangent to the radius of the mirror, attached at x=0. The
fixed mirror is 150 mm long, and extends along y.

notes: I estimated 1m for both the 3.5 and 2.5 secondary mirrors.
"""


import numpy
import math
import collections
import itertools
import mirror


mmPerMeter = 1000
MMPerInch = 25.4
RadPerDeg = math.pi / 180.0


MirSetUp = collections.namedtuple("mirSetUp", "actList fixedList encList")

def fakeEncoders(pistonMirPos, pistonBasePos, transMirPos, transBasePos, dist=75):
    """Generate new mirror and base positions to simulate an offset encoder.
    Input
    -pistonMirPos: mirPos positions for pistons - [[x,y,z], n] where n is number of actuators, 
                    these will be offset radially in x and y by dist

    -pistonBasePos: basePos positions for piston actuators - [[x,y,z], n] where n is 
                    number of actuators, these will be offset radially in x and y by dist

    -transMirPos:  mirPos position for transvers actuators - [[x,y,z], m] 
                    where n is number of actuators, these will be offset vertically by dist
                    
    -transBasePos:  basePos position for transvers actuators - [[x,y,z], m] 
                    where n is number of actuators, these will be offset vertically by dist
                    
    -dist: in mm, default is 75mm.
    
    Output
    -pistMirAdj:  numpy array [[x,y,z], n] for xyz mir positions determined by dist for n actuators
    -pistBaseAdj: numpy array [[x,y,z], n] for xyz base positions determined by dist for n actuators
    -transMirAdj: numpy array [[x,y,z], m] for xyz mir positions determined by dist for m actuators
    -transBaseAdj: numpy array [[x,y,z], m] for xyz base positions determined by dist for m actuators
    
    Note: this won't work for the 3.5m tert.  It assumes that all piston actuators are aligned
    with + Z.
    """
    
    # offset in xy radially (away from center) for pistons
    # iterate over columns to get xyz's (so transpose it)
    
    newXY = []
    for mir in pistonMirPos.T:
        # only care about xy right now
        vec = mir[0:2] # xy
        mag = numpy.linalg.norm(vec)
        xNew = (numpy.abs(vec[0]) * (mag + dist) ) / mag
        yNew = (numpy.abs(vec[1]) * (mag + dist) ) / mag
        
        if vec[0] < 0:
            xNew = -xNew
        if vec[1] < 0:
            yNew = -yNew
            
        newXY.append([xNew, yNew])
    
    newXY = numpy.asarray(newXY, dtype=float).T
    pistMirAdj = numpy.vstack((newXY, pistonMirPos[2,:])) # z wasn't changed
    pistBaseAdj = numpy.vstack((newXY, pistonBasePos[2,:])) # z wasn't changed
    # translation encoders + Z offset 
    if transMirPos==None and transBasePos==None: # for 3.5m tert (no translation)
        return pistMirAdj, pistBaseAdj
    nAct = len(transMirPos.T)
    addZ = numpy.zeros((3,nAct))
    addZ[2,:] = dist
    transMirAdj = transMirPos + addZ
    transBaseAdj = transBasePos + addZ
    return pistMirAdj, pistBaseAdj, transMirAdj, transBaseAdj


##################################################################################
########################## Actuator Lists for 2.5m Primary #######################

prim25List = []  # all created MirSetUp objects will be appended to this as they are created
"""prim25List has the following mirSetUp's:
0: act - adj len
1: act - adj base
2: fixed actuator 6 - adj len
3: fixed actuator 6 - adj base
4: fixed actuator 5 - adj len
5: fixed actuator 5 - adj base
6: generated encoders - adj len
7: generated encoders - adj base

"""
# data from mir file
minMnt = numpy.array([-120000., -120000., -120000., -90000., -50000., -50000])
maxMnt = numpy.array([ 120000.,  120000.,  120000.,  90000.,  50000.,  50000])

mntOffset = numpy.array([11300.,  -650.,  5500., -1650., -6900., -6900])
mntScale  = numpy.array([15.696, 15.696, 15.696, 15.696,  33.22, 32.53])

mirX  = numpy.array([    0., -749.03,  749.03,     0.,     0.,    0.])
mirY  = numpy.array([864.90, -432.45, -432.45, -1305., -1277., 1277.])
mirZ  = numpy.array([  251.,    251.,    251.,   238.,   262.,  262.])
baseX = numpy.array([    0., -749.03,  749.03,     0.,  -698., -698.])
baseY = numpy.array([864.90, -432.45, -432.45,   -9e9, -1277., 1277.])
baseZ = numpy.array([   9e9,     9e9,     9e9,   238.,   262.,  262.])

mirXYZ  = numpy.vstack((mirX, mirY, mirZ))
baseXYZ = numpy.vstack((baseX, baseY, baseZ))



# transpose is to iterate over columns, not rows
mir25_prim_adjLen = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                           
prim25List.append(MirSetUp(mir25_prim_adjLen, [], []))

mir25_prim_adjBase = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

prim25List.append(MirSetUp(mir25_prim_adjBase, [], []))

# use each tangental transverse actuator as a fixed mirror   
fixed = mirror.FixedLengthLink(baseXYZ[:,5], mirXYZ[:,5]) # 6th actuator
prim25List.append(MirSetUp(mir25_prim_adjLen[0:5], [fixed], []))
prim25List.append(MirSetUp(mir25_prim_adjBase[0:5], [fixed], []))
    
fixed = mirror.FixedLengthLink(baseXYZ[:,4], mirXYZ[:,4]) # 5th actuator
prim25List.append(MirSetUp(mir25_prim_adjLen[0:5], [fixed], []))
prim25List.append(MirSetUp(mir25_prim_adjBase[0:5], [fixed], []))

# make a fake encoder list, 
pistMirAdj, pistBaseAdj, transMirAdj, transBaseAdj = fakeEncoders(mirXYZ[:,0:3], baseXYZ[:,0:3], 
                                                                   mirXYZ[:,3:], baseXYZ[:,3:])
                                     
mirXYZ  = numpy.hstack((pistMirAdj, transMirAdj)) # use computed coords
baseXYZ = numpy.hstack((pistBaseAdj, transBaseAdj))

mir25_prim_adjLen_enc = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

prim25List.append(MirSetUp(mir25_prim_adjLen, [], mir25_prim_adjLen_enc))

mir25_prim_adjBase_enc = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                          
prim25List.append(MirSetUp(mir25_prim_adjBase, [], mir25_prim_adjBase_enc))



##################################################################################
########################## Actuator Lists for 2.5m Secondary #####################
sec25List = []
"""sec25List has the following mirSetUp's:  All contain a fake z rotation fixed mirror
0: act - adj len
1: act - adj base
2: generated encoders - adj len
3: generated encoders - adj base 
"""
minMnt = numpy.array([-7250000., -7250000., -7250000., -18000., -18000])
maxMnt = numpy.array([ 7250000.,  7250000.,  7250000.,  18000.,  18000])

mntOffset = numpy.array([     0.,      0.,      0.,  1700., -1700.])
mntScale  = numpy.array([1259.84, 1259.84, 1259.84, 31.496, 31.496])

mirX  = numpy.array([ 293.81, -233.08,  -60.73,   19.80,  -19.80])
mirY  = numpy.array([  99.51,  204.69, -304.20,  -19.80,  -19.80])
mirZ  = numpy.array([-193.00, -193.00, -193.00, -263.80, -263.80])
baseX = numpy.array([ 293.81, -233.08,  -60.73,   56.57,  -56.57])
baseY = numpy.array([  99.51,  204.69, -304.20,  -56.57,  -56.57])
baseZ = numpy.array([-280.00, -280.00, -280.00, -263.80, -263.80])

mirXYZ  = numpy.vstack((mirX, mirY, mirZ))
baseXYZ = numpy.vstack((baseX, baseY, baseZ))

# create a fake FixedLengthLink to constrain z rotation, extending in x, length = 150 mm
linkLength = 150. #mm
mirPos = numpy.array([0., 1 * mmPerMeter, -193.])
basePos = numpy.array([linkLength, 1 * mmPerMeter, -193.])
fixed = mirror.FixedLengthLink(basePos, mirPos)

# transpose is to iterate over columns, not rows
mir25_sec_adjLen = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                           
sec25List.append(MirSetUp(mir25_sec_adjLen, [fixed], []))

mir25_sec_adjBase = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

sec25List.append(MirSetUp(mir25_sec_adjBase, [fixed], []))
    
# make a fake encoder list, 
pistMirAdj, pistBaseAdj, transMirAdj, transBaseAdj = fakeEncoders(mirXYZ[:,0:3], baseXYZ[:,0:3], 
                                                                   mirXYZ[:,3:], baseXYZ[:,3:])
                                     
mirXYZ  = numpy.hstack((pistMirAdj, transMirAdj)) # use computed coords
baseXYZ = numpy.hstack((pistBaseAdj, transBaseAdj))

mir25_sec_adjLen_enc = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

sec25List.append(MirSetUp(mir25_sec_adjLen, [fixed], mir25_sec_adjLen_enc))

mir25_sec_adjBase_enc = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                          
sec25List.append(MirSetUp(mir25_sec_adjBase, [fixed], mir25_sec_adjBase_enc))

    
##################################################################################
########################## Actuator Lists for 3.5m Secondary #####################
sec35List = []
"""sec35List has the following mirSetUp's:  All contain a fake z rotation fixed mirror
0: act - adj len
1: act - adj base
2: generated encoders - adj len
3: generated encoders - adj base 
"""
minMnt = numpy.array([-7250000., -7250000., -7250000., -95000., -95000])
maxMnt = numpy.array([ 7250000.,  7250000.,  7250000.,  95000.,  95000])

mntOffset = numpy.array([      0.,       0.,       0.,     0.,     0.])
mntScale  = numpy.array([1259.843, 1259.843, 1259.843, 31.496, 31.496])

mirX  = numpy.array([      0., -230.529,  230.529,  29.186,   -29.186])
mirY  = numpy.array([ 266.192, -133.096, -133.096,  29.186,    29.186])
mirZ  = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
baseX = numpy.array([      0., -230.529,  230.529,  284.010, -284.010])
baseY = numpy.array([ 266.192, -133.096, -133.096,  284.010,  284.010])
baseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])

mirXYZ  = numpy.vstack((mirX, mirY, mirZ))
baseXYZ = numpy.vstack((baseX, baseY, baseZ))

# Fake FixedLengthLink
linkLength = 150. #mm
mirPos = numpy.array([0., 1 * mmPerMeter, -152.806])
basePos = numpy.array([linkLength, 1 * mmPerMeter, -152.806])
fixed = mirror.FixedLengthLink(basePos, mirPos)

# transpose is to iterate over columns, not rows
mir35_sec_adjLen = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                           
sec35List.append(MirSetUp(mir35_sec_adjLen, [fixed], []))

mir35_sec_adjBase = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

sec35List.append(MirSetUp(mir35_sec_adjBase, [fixed], []))  

# make a fake encoder list, 
pistMirAdj, pistBaseAdj, transMirAdj, transBaseAdj = fakeEncoders(mirXYZ[:,0:3], baseXYZ[:,0:3], 
                                                                   mirXYZ[:,3:], baseXYZ[:,3:])
                                     
mirXYZ  = numpy.hstack((pistMirAdj, transMirAdj)) # use computed coords
baseXYZ = numpy.hstack((pistBaseAdj, transBaseAdj))

mir35_sec_adjLen_enc = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

sec35List.append(MirSetUp(mir35_sec_adjLen, [fixed], mir35_sec_adjLen_enc))

mir35_sec_adjBase_enc = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ.T, mirXYZ.T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                          
sec35List.append(MirSetUp(mir35_sec_adjBase, [fixed], mir35_sec_adjBase_enc))

##################################################################################
########################## Actuator Lists for 3.5m Tert ##########################
tert35List=[]
"""tert35List has the following mirSetUp's:  All contain 3 fixed length links.
0: act - adj len
1: act - adj base
2: enc - adj len, true fixed link list
3: enc - adj base, true fixed link list

"""
minMnt = numpy.array([-7250000., -7250000., -7250000])
maxMnt = numpy.array([ 7250000.,  7250000.,  7250000])

mntOffset = numpy.array([      0.,       0.,       0.])
mntScale  = numpy.array([1259.843, 1259.843, 1259.843])

mirX  = numpy.array([      0.,  257.673, -257.673,        0.,        0.,       298.])
mirY  = numpy.array([-226.105,   89.479,   89.479,        0.,        0.,         0.])
mirZ  = numpy.array([ 194.674, -120.910, -120.910,        0.,        0.,         0.]) 
baseX = numpy.array([      0.,  257.673, -257.673,     1e+09,    -1e+09,       298.])
baseY = numpy.array([-271.006,   44.578,   44.578,  7.07e+08,  7.07e+08,   7.07e+08])
baseZ = numpy.array([ 149.773, -165.811, -165.811, -7.07e+08,  -7.07e+08, -7.07e+08])

mirXYZ  = numpy.vstack((mirX,   mirY,  mirZ))
baseXYZ = numpy.vstack((baseX, baseY, baseZ))

# last 3 are fixed links
fixed = [mirror.FixedLengthLink(base, mir) for base, mir in itertools.izip(baseXYZ[:,3:].T, mirXYZ[:,3:].T)]
# transpose is to iterate over columns, not rows, only loop over first 3 actuators (others are fix).
mir35_tert_adjLen = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ[:,0:3].T, mirXYZ[:,0:3].T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                           
tert35List.append(MirSetUp(mir35_tert_adjLen, fixed, []))

mir35_tert_adjBase = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ[:,0:3].T, mirXYZ[:,0:3].T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                           
tert35List.append(MirSetUp(mir35_tert_adjBase, fixed, []))

#### following computations copied/adjusted from russell's 3.5 M3 Actuator Positions 2008-04-18.py

#rad =   11.714 * MMPerInch
# changing rad to get offset encoders
rad =   (11.714 + 5.)* MMPerInch
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
for actInd, angRad in enumerate(angRadList):
    mirIP[actInd, :] = numpy.array((
        math.cos(angRad) * rad,
        math.sin(angRad) * rad,
        zMir
    ))
    baseIP[actInd, 0:2] = mirIP[actInd, 0:2]
    baseIP[actInd, 2] = zBase
    

# now add constraints (from Nick Mac model):
# 3,4 are parallel
# 5 is perp to 3,4
mirIP[3] = (-203.2, 0., 0.)
mirIP[4] = (203.2, 0., 0.)
mirIP[5] = (0., 0., 0.)
baseIP[3] = mirIP[3].copy()
baseIP[3, 1] = (281.47)
baseIP[4] = mirIP[4].copy()
baseIP[4, 1] = (281.47) # positive y, base pivot is close to instrument port
baseIP[5] = (281.47, 0., 0.)

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

mirXYZ = numpy.dot(mirIP, rotMat).T
baseXYZ = numpy.dot(baseIP, rotMat).T

# generate new fixed links (not infinite)
fixed = [mirror.FixedLengthLink(base, mir) for base, mir in itertools.izip(baseXYZ[:,3:].T, mirXYZ[:,3:].T)]

# generate encoder list
mir25_sec_adjLen_enc = [mirror.AdjLengthLink(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ[0:3].T, mirXYZ[0:3].T, minMnt, maxMnt,
                                          mntScale, mntOffset)]

tert35List.append(MirSetUp(mir35_tert_adjLen, fixed, mir25_sec_adjLen_enc))


mir25_sec_adjBase_enc = [mirror.AdjBaseActuator(base, mir, min, max, scale, off) 
                        for base, mir, min, max, scale, off 
                        in itertools.izip(baseXYZ[0:3].T, mirXYZ[0:3].T, minMnt, maxMnt,
                                          mntScale, mntOffset)]
                                          
tert35List.append(MirSetUp(mir35_tert_adjBase, fixed, mir25_sec_adjBase_enc))

