if [ $1 = cam1 ]
then
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video1 -r 320x240" -o "/usr/local/lib/output_http.so -p 8001 -w /usr/local/www"
fi

if [ $1 = cam0 ]
then
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video0 -r 320x240" -o "/usr/local/lib/output_http.so -p 8000 -w /usr/local/www" 
fi

