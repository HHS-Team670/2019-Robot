if [ $1 = cam0 ]
then
#sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/v4l/by-id/usb-Microsoft_Microsoft®_LifeCam_HD-3000-video-index0 -r 320x240" -o "/usr/local/lib/output_http.so -p 80 -w /usr/local/www" &
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video0 -r 320x240" -o "/usr/local/lib/output_http.so -p 80 -w /usr/local/www" &
fi

if [ $1 = cam1 ]
then
#sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/v4l/by-id/usb-HD_Camera_Manufacturer_USB_2.0_Camera-video-index0 -r 320x240" -o "/usr/local/lib/output_http.so -p 80 -w /usr/local/www" &
sudo /usr/local/bin/mjpg_streamer -i "/usr/local/lib/input_uvc.so -d /dev/video1 -r 320x240" -o "/usr/local/lib/output_http.so -p 80 -w /usr/local/www" &
fi


