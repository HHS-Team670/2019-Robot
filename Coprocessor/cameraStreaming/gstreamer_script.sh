#!/bin/bash

CAMERA_FRONT="/dev/video40"
CAMERA_BACK="/dev/video41"
PORT_FOR_FRONT=80
PORT_FOR_BACK=443
DS_IP_ADDR=10.6.70.163
SELF_IP_ADDR=0.0.0.0

#while true
#do
	if [ -e ${CAMERA_FRONT} ]; then
	       ping -c 1 -w 1 ${DS_IP_ADDR}
	       if [ $? -eq 0 ]; then
		       gst-launch-1.0 -v v4l2src device=${CAMERA_FRONT} ! "video/x-raw,width=320,height=240,framerate=30/1" ! avenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=${DS_IP_ADDR} port=${PORT_FOR_FRONT} &
	       else 
			echo DS not found
	       fi
	else
		echo Camera not found
	fi
	
	if [ -e ${CAMERA_BACK} ]; then
	       ping -c 1 -w 1 ${DS_IP_ADDR}
	       if [ $? -eq 0 ]; then
			gst-launch-1.0 -v v4l2src device=${CAMERA_BACK} ! "video/x-raw,width=320,height=240,framerate=30/1" ! avenc_mpeg4 ! rtpmp4vpay config-interval=3 ! udpsink host=${DS_IP_ADDR} port=${PORT_FOR_BACK} &
	       else 
			echo DS not found
	       fi
	else
		echo Camera not found
	fi
	
#	sleep 5
#done
