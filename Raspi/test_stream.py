import datetime
from threading import Thread
import cv2
import imutils
import argparse

class FPS:
    def __init__(self):
        '''Store the start time, end time, and number of frames
        that were examined between start and end intervals
        '''
        self._start = None
        self._end = None
        self._numFrames = 0

    def start(self):
        '''start timer'''
        self._start = datetime.datetime.now()
        return self

    def stop(self):
        '''stop timer'''
        self._end = datetime.datetime.now()

    def update(self):
        '''increment frames examined'''
        self._numFrames += 1

    def elapsed(self):
        '''return total seconds between start and end'''
        return (self._end - self._start).total_seconds()

    def fps(self):
        '''compute approximate fps'''
        return self._numFrames / self.elapsed()

class WebcamVideoStream:
    def __init__(self, src=0):
        '''read first video stream frame'''
        self.stream = cv2.VideoCapture(src)
        self.grabbed = self.stream.read()
        self.frame=self.grabbed
        '''whether to be stopped'''
        self.stopped = False

    def start(self):
        Thread(target=self.update, args=()).start()
        return self

    def update(self):
        while True:
            '''stop thread'''
            if self.stopped:
                return
            self.grabbed = self.stream.read()
            self.frame=self.grabbed

    def read(self):
        '''recent frame'''
        return self.frame

    def stop(self):
        '''indicate stop of thread'''
        self.stopped = True

ap = argparse.ArgumentParser()
ap.add_argument("-n", "--num-frames", type=int, default=400,
	help="# of frames to loop over for FPS test")
ap.add_argument("-d", "--display", type=int, default=1,
	help="Whether or not frames should be displayed")
args = vars(ap.parse_args())
'''
##(without threading, 400 frames total)
##remove 3line string to see without threading (about 24 fps)

print("[INFO] sampling frames from webcam...")
stream = cv2.VideoCapture(0)
fps = FPS().start()
 
# loop over some frames
while fps._numFrames < args["num_frames"]:
	# grab the frame from the stream and resize it to have a maximum
	# width of 400 pixels
	(grabbed, frame) = stream.read()
	frame = imutils.resize(frame, width=400)
 
	# check to see if the frame should be displayed to our screen
	if args["display"] > 0:
		cv2.imshow("Frame", frame)
		key = cv2.waitKey(1) & 0xFF
 
	# update the FPS counter
	fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
stream.release()
cv2.destroyAllWindows()
'''
##with threading 400 frames total, about 200 fps

print("[INFO] sampling THREADED frames from webcam...")
vs = WebcamVideoStream(src=0).start()
fps = FPS().start()
 
# loop over some frames...this time using the threaded stream
while fps._numFrames < args["num_frames"]:
    # grab the frame from the threaded video stream and resize it
    # to have a maximum width of 400 pixels
    frame = vs.read()[1]
    frame=cv2.resize(frame,(0,0),fx=1,fy=1)
    frame = imutils.resize(frame, width=400)
    if args["display"] > 0:
        cv2.imshow("Frame", frame)
        key = cv2.waitKey(1) & 0xFF

    # update the FPS counter
    fps.update()
 
# stop the timer and display FPS information
fps.stop()
print("[INFO] elasped time: {:.2f}".format(fps.elapsed()))
print("[INFO] approx. FPS: {:.2f}".format(fps.fps()))
 
# do a bit of cleanup
cv2.destroyAllWindows()
vs.stop()
