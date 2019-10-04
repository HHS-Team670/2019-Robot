**Instructions for camera streaming with mjpg_streamer**
=================================

#update and install tools
- run `sudo apt update` and `sudo apt upgrade -y`
- run `sudo apt-get install build-essential libjpeg8-dev imagemagick libv4l-dev cmake -y`

#clone the mjpg-streamer repo
- `cd` to a convenient directory
- run `git clone https://github.com/jacksonliam/mjpg-streamer.git` then `cd mjpg-streamer/mjpg-streamer-experimental`

#add a symbolic link so the system knows where to find this particular file
- `sudo ln -s /usr/include/linux/videodev2.h /usr/include/linux/videodev.h`

#run make commands
- run `make`
- run `sudo make install`

#start the camera stream
- run mjpg_streamer_server.sh after cloning it and making changes to suit your needs (refer to https://github.com/jacksonliam/mjpg-streamer for detailed documentation on specifying inputs and outputs)
- *note:* all USB webcams should show up when `ls -l /dev/` is run --> make sure the cameras specified in the code actually exist by checking the contents of the /dev folder

**Instructions for running network tables**
================================

#install pynetworktables

#run the python program
- run `python nt_server.py` after cloning it and making changes to suit your needs (refer to [pynetworktables page] for detailed docs)
