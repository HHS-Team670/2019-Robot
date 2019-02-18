#!/usr/bin/env python3
#
# This is a NetworkTables server (eg, the robot or simulator side).
#
# On a real robot, you probably would create an instance of the
# wpilib.SmartDashboard object and use that instead -- but it's really
# just a passthru to the underlying NetworkTable object.
#
# When running, this will continue incrementing the value 'robotTime',
# and the value should be visible to networktables clients such as
# SmartDashboard. To view using the SmartDashboard, you can launch it
# like so:
#
#     SmartDashboard.jar ip 127.0.0.1
#

import time
from networktables import NetworkTables
import os
import subprocess
import threading

cond = threading.Condition()
notified = [False]

def connectionListener(connected, info):
    print(info, '; Connected=%s' % connected)
    with cond:
        notified[0] = True
        cond.notify()

NetworkTables.initialize(server='10.6.70.2')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

#NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")
VISION_ERROR_CODE = -9999

os.system('/home/pi/git/Mustang-Pi/cameraStreaming/mjpg_streamer_server.sh cam1 &')

cam = '0'
def valueChanged(table, key, value, isNew):
    global cam
    if (key=='camera-source'):
        os.system('sudo killall mjpg_streamer')
        cam = int(cam)
        numCams = int(os.popen('ls -l /dev/ | egrep video.$ | wc -l').read().replace('\n', ''))
        cam = (cam + 1) % numCams
        cam = str(cam)
        sd.putString('camera-source', '')
        os.system('/home/pi/git/Mustang-Pi/cameraStreaming/mjpg_streamer_server.sh cam' + cam + ' &')	

i = 0

sd.addEntryListener(valueChanged)

while True:
    sd.putString('vision-status', "none")
    if (int(os.popen('ls -l /dev/ | egrep video.$ | wc -l').read().replace('\n', '')) == 0):
        sd.putString('warnings', 'no cameras found')
        sd.putString('vision-status', str(VISION_ERROR_CODE))
