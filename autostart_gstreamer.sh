# TODO replace \\mnt\\c\\gstreamer... with the actual path of gstreamer on the DS laptop

echo "starting...."

C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=80 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &
C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=443 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &
