import numpy
import math
import link
################ 2.5m Primary #######################

PrimMinMount = numpy.array([-120000., -120000., -120000., -90000., -50000., -50000])
PrimMaxMount = numpy.array([120000., 120000., 120000., 90000., 50000., 50000])

PrimMountOffset = numpy.array([11300., -650., 5500., -1650., -6900., -6900])
PrimMountScale = numpy.array([15.696, 15.696, 15.696, 15.696, 33.22, 32.53])

PrimActMirX = numpy.array([0., -749.03, 749.03, 0., 0., 0.])
PrimActMirY = numpy.array([864.90, -432.45, -432.45, -1305., -1277., 1277.])
PrimActMirZ = numpy.array([251., 251., 251., 238., 262., 262.])
PrimActBaseX = numpy.array([0., -749.03, 749.03, 0., -698., -698.])
PrimActBaseY = numpy.array([864.90, -432.45, -432.45, -9e9, -1277., 1277.])
PrimActBaseZ = numpy.array([9e9, 9e9, 9e9, 238.00, 262., 262.00])

PrimActMirXYZ = numpy.vstack((PrimActMirX, PrimActMirY, PrimActMirZ))
PrimActBaseXYZ = numpy.vstack((PrimActBaseX, PrimActBaseY, PrimActBaseZ))

# make actuator list

mir25_prim_actList = []
for k in range(len(PrimMinMount)):
#   basePos, mirPos, minMount, maxMount, scale
    act = link.AdjLengthLink(PrimActBaseXYZ[:,k], PrimActMirXYZ[:,k], PrimMinMount[k], 
                                 PrimMaxMount[k], PrimMountScale[k], PrimMountOffset[k])
    mir25_prim_actList.append(act)

# make a version using a fixed link for rotZ    
mir25_prim_actListFixed = mir25_prim_actList[0:5]  # take first 5
fixed = link.FixedLengthLink(PrimActBaseXYZ[:,5], PrimActMirXYZ[:,5])
mir25_prim_actListFixed.append(fixed)
# make a fake encoder list, 
# by dist (mm)
dist = 75 #mm about 3in according to Nick

PrimActMirXYZ_e = PrimActMirXYZ.copy()
PrimActBaseXYZ_e = PrimActBaseXYZ.copy()

# offset in [0:3] offset radially (away from center) for pistons
for k in range(3):
    vec = numpy.asarray(PrimActMirXYZ_e[0:2,k])
    mag = math.sqrt(numpy.dot(vec, vec))
    xNew = (numpy.abs(vec[0]) * (mag + dist) ) / mag
    yNew = (numpy.abs(vec[1]) * (mag + dist) ) / mag
    
    if vec[0] < 0:
        xNew = -xNew
    if vec[1] < 0:
        yNew = -yNew
        
    PrimActMirXYZ_e[0:2,k] = [xNew, yNew]
    PrimActBaseXYZ_e[0:2,k] = [xNew, yNew]
    
    
# translation encoders + Z offset    
PrimActMirXYZ_e[2,3:5] = PrimActMirXYZ_e[2,3:5] - dist  # - sign puts encoder below actuator
PrimActBaseXYZ_e[2,3:5] = PrimActBaseXYZ_e[2,3:5] - dist    

mir25_prim_encList = []
for k in range(len(PrimMinMount)):  
#   basePos, mirPos, minMount, maxMount, scale
    act = link.AdjLengthLink(PrimActBaseXYZ_e[:,k], PrimActMirXYZ_e[:,k], PrimMinMount[k], 
                                 PrimMaxMount[k], PrimMountScale[k], PrimMountOffset[k])
    mir25_prim_encList.append(act)
    



############### 2.5m Secondary #############################

SecMinMount = numpy.array([-7250000., -7250000., -7250000., -18000., -18000])
SecMaxMount = numpy.array([7250000., 7250000., 7250000., 18000., 18000])

SecMountOffset = numpy.array([0., 0., 0., 1700., -1700.])
SecMountScale = numpy.array([1259.84, 1259.84, 1259.84, 31.496, 31.496])

SecActMirX = numpy.array([293.81, -233.08, -60.73, 19.80, -19.80])
SecActMirY = numpy.array([99.51, 204.69, -304.20, -19.80, -19.80])
SecActMirZ = numpy.array([-193.00, -193.00, -193.00, -263.80, -263.80])
SecActBaseX = numpy.array([293.81, -233.08, -60.73, 56.57, -56.57])
SecActBaseY = numpy.array([99.51, 204.69, -304.20, -56.57, -56.57])
SecActBaseZ = numpy.array([-280.00, -280.00, -280.00, -263.80, -263.80])

SecActMirXYZ = numpy.vstack((SecActMirX, SecActMirY, SecActMirZ))
SecActBaseXYZ = numpy.vstack((SecActBaseX, SecActBaseY, SecActBaseZ))
    
mir25_sec_actList = []
for k in range(len(SecMinMount)):
#   basePos, mirPos, minMount, maxMount, scale
    act = link.AdjLengthLink(SecActBaseXYZ[:,k], SecActMirXYZ[:,k], SecMinMount[k], 
                                 SecMaxMount[k], SecMountScale[k], SecMountOffset[k])
    mir25_sec_actList.append(act)
    
# create a fake FixedLengthLink to constrain z rotation, extending in x, length = 150 mm

mmPerMeter = 1000
mirPos = numpy.array([0., 1 * mmPerMeter, -193.])
basePos = numpy.array([150., 1 * mmPerMeter, -193.])
act = link.FixedLengthLink(basePos, mirPos)
mir25_sec_actList.append(act)
    
##################### 3.5m Secondary #####################

SecMinMount = numpy.array([-7250000., -7250000., -7250000., -95000., -95000])
SecMaxMount = numpy.array([7250000., 7250000., 7250000., 95000., 95000])

SecMountOffset = numpy.array([0., 0., 0., 0., 0.])
SecMountScale = numpy.array([1259.843, 1259.843, 1259.843, 31.496, 31.496])

SecActMirX = numpy.array([0.000, -230.529, 230.529, 29.186, -29.186])
SecActMirY = numpy.array([266.192, -133.096, -133.096, 29.186, 29.186])
SecActMirZ = numpy.array([-152.806, -152.806, -152.806, -167.361, -167.361])
SecActBaseX = numpy.array([0.000, -230.529, 230.529, 284.010, -284.010])
SecActBaseY = numpy.array([266.192, -133.096, -133.096, 284.010, 284.010])
SecActBaseZ = numpy.array([-256.438, -256.438, -256.438, -192.710, -192.710])

SecActMirXYZ = numpy.vstack((SecActMirX, SecActMirY, SecActMirZ))
SecActBaseXYZ = numpy.vstack((SecActBaseX, SecActBaseY, SecActBaseZ))
    
mir35_sec_actList = []
for k in range(len(SecMinMount)):
#   basePos, mirPos, minMount, maxMount, scale
    act = link.AdjLengthLink(SecActBaseXYZ[:,k], SecActMirXYZ[:,k], SecMinMount[k], 
                                 SecMaxMount[k], SecMountScale[k], SecMountOffset[k])
    mir35_sec_actList.append(act)

# create a fake FixedLengthLink to constrain z rotation, extending in x, length = 150 mm

mmPerMeter = 1000
mirPos = numpy.array([0., 1 * mmPerMeter, -152.806])
basePos = numpy.array([150., 1 * mmPerMeter, -152.806])
act = link.FixedLengthLink(basePos, mirPos)
mir35_sec_actList.append(act)

###################### 3.5 Tertiary ##############################

TertMinMount = numpy.array([-7250000., -7250000., -7250000])
TertMaxMount = numpy.array([7250000., 7250000., 7250000])

TertMountOffset = numpy.array([0., 0., 0.])
TertMountScale = numpy.array([1259.843, 1259.843, 1259.843])

TertActMirX = numpy.array([0.000, 257.673, -257.673, 0., 0., 298])
TertActMirY = numpy.array([-226.105, 89.479, 89.479, 0., 0., 0])
TertActMirZ = numpy.array([194.674, -120.910, -120.910, 0., 0., 0]) 
TertActBaseX = numpy.array([0.000, 257.673, -257.673, 1e+09, -1e+09, 298])
TertActBaseY = numpy.array([-271.006, 44.578, 44.578, 7.07e+08, 7.07e+08, 7.07e+08])
TertActBaseZ = numpy.array([149.773, -165.811, -165.811, -7.07e+08, -7.07e+08, -7.07e+08])

TertActMirXYZ = numpy.vstack((TertActMirX, TertActMirY, TertActMirZ))
TertActBaseXYZ = numpy.vstack((TertActBaseX, TertActBaseY, TertActBaseZ))

mir35_tert_actList = []
for k in range(len(TertMinMount)):
#   basePos, mirPos, minMount, maxMount, scale
    act = link.AdjLengthLink(TertActBaseXYZ[:,k], TertActMirXYZ[:,k], TertMinMount[k], 
                             TertMaxMount[k], TertMountScale[k], TertMountOffset[k])
    mir35_tert_actList.append(act)

#last 3 actuators are fixed links    
for k in range(len(TertMinMount),len(TertActMirX)):
#   basePos, mirPos
    act = link.FixedLengthLink(TertActBaseXYZ[:,k], TertActMirXYZ[:,k])
    mir35_tert_actList.append(act)

