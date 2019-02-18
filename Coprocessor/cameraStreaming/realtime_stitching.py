# import the necessary packages
from __future__ import print_function
from panorama import Stitcher
from imutils.video import VideoStream
from imutils.video import WebcamVideoStream
import numpy as np
import datetime
import imutils
import time
import cv2
#import cv2.xfeatures2d.SIFT_create()
#import cv2.xfeatures2d.SURF_create()

# initialize the video streams and allow them to warmup
print("[INFO] starting cameras...")
#leftStream = VideoStream(src=2).start()
#rightStream = VideoStream(src=1).start()
#time.sleep(2.0)
leftVideo = cv2.VideoCapture(0)
print("left started")
rightVideo = cv2.VideoCapture(1)
print("right started")
leftVideo.set(3,320);
leftVideo.set(4,240);
rightVideo.set(3,320);
rightVideo.set(4,240);



print("before init")
# initialize the image stitcher, motion detector, and total
# number of frames read
stitcher = Stitcher()
total = 0
print("after init")

# loop over frames from the video streams
while True:
        print("...")
	# grab the frames from their respective video streams
	#left = leftStream.read()
	#right = rightStream.read()
        #print("left: ", left)
        #print("right: ", right)
	# resize the frames
	#left = imutils.resize(left, width=400)
	#right = imutils.resize(right, width=400)

        leftImage = leftVideo.read()[1]
        rightImage = rightVideo.read()[1]
        print("left:", leftImage)
        print("right:", rightImage)
        cv2.imwrite("left.jpg", leftImage)
        cv2.imwrite("right.jpg", rightImage)

        print("getting")
        result = stitcher.stitch([leftImage, leftImage])
        print("got")

	# stitch the frames together to form the panorama
	# IMPORTANT: you might have to change this line of code
	# depending on how your cameras are oriented; frames
	# should be supplied in left-to-right order
	print("getting result")
        #result = stitcher.stitch([left, right])
        print("got result")


	# no homograpy could be computed
	if result is None:
		print("[INFO] homography could not be computed")
		break

	# convert the panorama to grayscale, blur it slightly, update
	# the motion detector
	gray = cv2.cvtColor(result, cv2.COLOR_BGR2GRAY)
	gray = cv2.GaussianBlur(gray, (21, 21), 0)

        # only process the panorama for motion if a nice average has
	# been built up
	#if total > 32 and len(locs) > 0:
		# initialize the minimum and maximum (x, y)-coordinates,
		# respectively
	#	(minX, minY) = (np.inf, np.inf)
	#	(maxX, maxY) = (-np.inf, -np.inf)

		# loop over the locations of motion and accumulate the
		# minimum and maximum locations of the bounding boxes
	#	for l in locs:
	#		(x, y, w, h) = cv2.boundingRect(l)
	#		(minX, maxX) = (min(minX, x), max(maxX, x + w))
	#		(minY, maxY) = (min(minY, y), max(maxY, y + h))

		# draw the bounding box
	#	cv2.rectangle(result, (minX, minY), (maxX, maxY),
	#		(0, 0, 255), 3)

                # increment the total number of frames read and draw the
	# timestamp on the image
	#total += 1
	#timestamp = datetime.datetime.now()
	#ts = timestamp.strftime("%A %d %B %Y %I:%M:%S%p")
	#cv2.putText(result, ts, (10, result.shape[0] - 10),
	#	cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1)

	# show the output images
	cv2.imshow("Result", result)
	cv2.imshow("Left Frame", left)
	cv2.imshow("Right Frame", right)
	path = '/home/pi/git/Mustang-Pi/cameraStreaming/'
        cv2.imwrite(os.path.join(path , 'stitched.mjpg'), result)
        
        key = cv2.waitKey(1) & 0xFF

	# if the `q` key was pressed, break from the loop
	if key == ord("q"):
		break

# do a bit of cleanup
print("[INFO] cleaning up...")
cv2.destroyAllWindows()
leftStream.stop()
rightStream.stop()

class ThreadedVideo:
    '''This class creates a thread for video capturing. self.stream is the capture stream'''
    def __init__(self, resize_value, src):
        '''
        Init class that initializes a video stream and a single
        video capture image.
        '''
        self.stream = cv2.VideoCapture(src)
        self.width = 1000
        self.grabbed = (read_video_image(self.stream, resize_value), round(time.time()*1000))
        self.frame = imutils.resize(self.grabbed[0], width=self.width)
        self.stopped = False
        self.resize = resize_value

    def start(self):
        '''Starts the thread for video processing.'''
        self.thread = Thread(target=self.update, args=())
        self.thread.start()
        return self

    def stop(self):
        '''
        Stops the thread for video processing. For some reason the
        entire program crashes after the q key is pressed to quit,
        but it doesn't affect anything sooooo who cares
        '''
        self.stopped = True

    def read(self):
        '''Returns the resized frame. Unnecessary for the actual robot'''
        return self.frame

    def raw_read(self):
        '''Returns the raw frame with the original video input's image size.'''
        return self.grabbed

    def update(self):
        '''Grabs new video images from the current video stream.'''
        while not self.stopped:
            if not self.stopped:
                self.grabbed = (read_video_image(self.stream, self.resize), round(time.time()*1000))
           #     if self.grabbed[0] is not None:
           #         self.frame = imutils.resize(self.grabbed[0], width=self.width)
