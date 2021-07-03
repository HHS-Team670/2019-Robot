import os
from subprocess import call
os.system('cd C:\\Users\\Driver\\Documents\\GitHub\\FRCDashboard')
os.system('START "" npm start &')
os.system('START "" C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=80 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &')
os.system('START "" C:\\gstreamer\\1.0\\x86_64\\bin\\gst-launch-1.0.exe udpsrc port=443 caps="application/x-rtp" ! rtpmp4vdepay ! avdec_mpeg4 ! fpsdisplaysink &')

