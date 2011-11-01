
import numpy
import warnings
import time
import matplotlib.pyplot
import mirdef


reload(mirdef)

# creates actuator lists for all APO mirrors
# mir25_prim_actList -- direct
# mir25_sec_actList -- tip-trans, only 5 actuators defined
# mir35_sec_actList -- direct, only 5 actuators
# mir35_tert_actList -- direct with 3 fixed links
execfile('parseMirFile.py')

# using old actuator geometry
actList = mir25_prim_actList
encList = mir25_prim_encList

actListFix = mir25_prim_actListFixed


orientTest = [0,0,0,100,0,100] 

print '-------------------------- no fixed link -------------------------'
Mir = mirdef.DirectMirror(actList)
mount1 = Mir.orient2Mount(orientTest)
print 'mount1: ', mount1
t1=time.time()
orient1 = Mir.mount2Orient(mount1)
print 'time: ', time.time() - t1

print 'orient1: ', orient1
print ' '
print '-------------------------- fixed link -------------------------'

Mir = mirdef.DirectMirrorZFix(actListFix)
mount2 = Mir.orient2Mount(orientTest)
print 'mount2 : ', mount2
t1=time.time()
orient2 = Mir.mount2Orient(mount2)
print 'time: ', time.time() - t1

print 'orient2: ', orient2

mount3 = Mir.orient2Mount(orient2)
print 'mount3: ', mount3

