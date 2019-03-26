sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video41 -f 5 -r 320x240" -o "/usr/local/lib/output_http.so -p 443 -w /usr/local/www" &
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video40 -f 5 -r 320x240" -o "/usr/local/lib/output_http.so -p 80 -w /usr/local/www" &
