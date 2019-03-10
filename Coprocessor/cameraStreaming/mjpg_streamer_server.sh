sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.2:1.0-video-index0 /dev/video40

sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video41 -r 320x240" -o "/usr/local/lib/output_http.so -p 8001 -w /usr/local/www" &
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video40 -r 320x240" -o "/usr/local/lib/output_http.so -p 8000 -w /usr/local/www" &
