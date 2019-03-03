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
#NetworkTables.initialize()
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

with cond:
    print("Waiting")
    if not notified[0]:
        cond.wait()

#NetworkTables.initialize()
sd = NetworkTables.getTable("SmartDashboard")
VISION_ERROR_CODE = -9999

#os.system('sudo python /home/pi/git/Mustang-Pi/cameraStreaming/watchdog.py "/home/pi/git/Mustang-Pi/cameraStreaming/mjpg_streamer_server.sh > /tmp/error_cams 2>&1" &')   

os.system('sudo /home/pi/git/Coprocessor/cameraStreaming/mjpg_streamer_server.sh')

cam = '0'
def valueChanged(table, key, value, isNew):
    global cam
    if (key=='camera-source'):
        ids = os.popen('sudo ps -aef | grep watchdog | awk \'{print $2}\'').read().split('\n')
        for i in ids:
            print ('>>> id: ', i)
            if i != ids[len(ids)-1]:
                os.system('sudo kill -9 ' + i) 
        stream_ids = os.popen('sudo ps -aef | grep mjpg | awk \'{print $2}\'').read().split('\n')
        for i in stream_ids:
            print('>>> stream id: ', i)
            if i != stream_ids[len(stream_ids)-1]:
                os.system('sudo kill -9 ' + i)
        
        cam = value
        os.system('sudo python /home/pi/git/Mustang-Pi/cameraStreaming/watchdog.py "/home/pi/git/Mustang-Pi/cameraStreaming/mjpg_streamer_server.sh cam' + cam + ' > /tmp/error' + cam + ' 2>&1" &')   

i = 0

#sd.addEntryListener(valueChanged)

while True:
    print(i)
    if (i%10 == 0):
        sd.putString("driver-camera-mode", "single")
    if (i%10 == 5):
        sd.putString("driver-camera-mode", "double")
    sd.putString('vision-status', "none")
    if (int(os.popen('ls -l /dev/ | egrep video.$ | wc -l').read().replace('\n', '')) == 0):
        sd.putString('warnings', 'no cameras found')
        sd.putString('vision-status', str(VISION_ERROR_CODE))
    i = i + 1
    time.sleep(1)
#    if (os.system('grep "cleaning up resources" /tmp/error0') == 0):
#       os.system('sudo kill $(ps -aef | grep 8000 | grep mjpg_streamer | grep sudo | awk "{print $2}"')
