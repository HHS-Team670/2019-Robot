sleep 10
cd /home/pi/git/Coprocessor/cameraStreaming && /usr/bin/python /home/pi/git/Coprocessor/cameraStreaming/nt_server.py & 
cd /home/pi/git/Coprocessor && /usr/bin/python /home/pi/git/Coprocessor/vision2019.py &

#redo symlinks for cameras
sudo rm -f /dev/video50
sudo rm -f /dev/video51
sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.5:1.0-video-index0' '/dev/video50
sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0' '/dev/video51
