sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video51 -r 320x240" -o "/usr/local/lib/output_http.so -p 8001 -w /usr/local/www" &
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video50 -r 320x240" -o "/usr/local/lib/output_http.so -p 8000 -w /usr/local/www" &

