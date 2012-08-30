"""Startup script to initialize a bunch of actor/twisted galil pairs
"""

import subprocess
import itertools

galilPorts = range(8000,8003)
actorPorts = range(1025,1028)

procs = []

def cleanup():
    """cleanup all processes
    """
    for proc in procs:
        proc.kill()

try:
    for port in galilPorts:
        # start up the fake Galils on their own ports
        procs.append(subprocess.Popen(["python", "twistedGalil.py", str(port)]))    
    for actPort, galPort in itertools.izip(actorPorts, galilPorts):
        procs.append(subprocess.Popen(["python", "actorTest.py", str(actPort), str(galPort)]))
except Exception as e:
    # clean up any started processes before exiting
    print e
    print 'cleaning up'
    cleanup()

while True:
    cmd = raw_input('q to quit')
    if cmd == 'q':
        print 'Shutting down ports'
        cleanup()
        break
    else:
        print 'enter "q" to quit'



