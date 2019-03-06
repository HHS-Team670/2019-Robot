'''
Arjun Sampath || Kyle Fu || Rishab Borah || Navaneet Kadaba || Eshan Jain || Harinandan K
Usage notes:
Set DEBUG_MODE to True in order to make screens appear showing what robot sees
Angle calculations are reliant on constants--the camera's FOV and focal length
Depth code has been moved to the roboRIO
Uses 2 cameras and automatically sets ports to video source 50 and 51 and resets the symlinks everytime the program runs
Sets timestamp to VISION_ERROR_CODE and vision-status to error on SmartDashboard if the camera being used is not connected
Automatically reconnects in code if unplug and replug camera
You can click on a point on the output image to print out the hsv value of
that point in debug mode. Useful for finding specific hsv color ranges.
Pushes an array (hangle, vangle, timestamp) to network tables--for PurePursuit and other Vision code to use
copper goes to negative
'''

import copy
import math
from threading import Thread
from threading import Condition
import cv2
import numpy as np
import time
from networktables import NetworkTables
import os

DEBUG_MODE = False # NOTE MAKE this @FALSE TO MAKE NO SCREENS APPEAR
ERROR = -99999

ROBORIO_IP = "10.6.70.2"
NETWORK_TABLE_NAME = "SmartDashboard"
NETWORK_KEY = "reflect_tape_vision_data"

# Variables (These should be changed to reflect the camera)
front_capture_source = 50 # Video source number for front camera
back_capture_source = 51 # Video source number for back camera
camera_fov_vertical = 39.7  # FOV of the camera (in degrees)
camera_fov_horizontal = 60.0
screen_resize = 1  # Scale that the GUI image should be scaled to
timestamp = round(time.time() * 1000) # time in milliseconds

# Keys for network table entries
camera_key = "vision-camera"
enabled_key = "vision-enabled"

# HSV Values to detect - DEFAULT is Front
min_hsv = [50, 220, 80] # Need to change according to practice field data
max_hsv = [70, 255, 210]

# Min area to make sure not to pick up noise
MIN_AREA = 100

# Network table (by default returns error codes, but changes in program)
returns = [ERROR, ERROR, timestamp]
cond = Condition()
notified = [False]

vs_front=None
vs_back=None
frames = 0

#Connection Listener for Network Tables
def connectionListener(connected, info):
    with cond:
        print("connected%s"%connected)
        notified[0] = True
        cond.notify()

def main():
    '''
    Main method, runs when program is run.
    '''
    #Resets the symlinks to vid source 50 and 51, connecting them to usb ports
    os.system("sudo rm -f /dev/video" + `front_capture_source`)
    os.system("sudo rm -f /dev/video" + `back_capture_source`)
    os.system("sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.5:1.0-video-index0 /dev/video"+ `front_capture_source`)
    os.system("sudo ln -s /dev/v4l/by-path/platform-3f980000.usb-usb-0:1.3:1.0-video-index0 /dev/video"+ `back_capture_source`)

    #Uncomment when saving images
    os.system("rm -rf output")
    os.system("mkdir output")
    
    #Initializes connection to RIO
    NetworkTables.initialize(server=ROBORIO_IP)

    NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)
    #Waits until RIO is connected before continuing
    with cond:
        if not notified[0]:
            print("waiting")
            cond.wait()

    table = NetworkTables.getTable(NETWORK_TABLE_NAME)
    table.addEntryListener(checkEnabled)

    #Video capture / resizing stuff
    global vs_front
    global vs_back
    vs_front = ThreadedVideo(screen_resize, front_capture_source).start()
    vs_back = ThreadedVideo(screen_resize, back_capture_source).start() 

    # This may not need to be calculated, can use Andra's precalculated values
    vert_focal_length = find_vert_focal_length(vs_front.raw_read()[0], camera_fov_vertical)
    hor_focal_length = find_hor_focal_length(vs_front.raw_read()[0], camera_fov_horizontal)

    global frames
    frames = 0

    while True:
        time.sleep(1)

def checkEnabled(table, key, value, isNew):

    if key == enabled_key and value != "enabled" or key != enabled_key:
        return

    # gets which camera to use (front or back)
    camera = table.getEntry(camera_key).getString("back")

    vs = None
    global vs_front
    global vs_back
    print(camera)

    if camera == "front":
        vs=vs_front
        #HSV Values to detect with front LED
        min_hsv = [50, 220, 80] # Need to change according to practice field data
        max_hsv = [70, 255, 210]
    elif camera == "back":
        vs=vs_back
        #HSV Values to detect with back LED
        min_hsv = [50, 220, 80] # Need to change according to practice field data
        max_hsv = [70, 255, 210]
    try:
        global frames
        frames += 1
        # Read input image from video
        input_raw = vs.raw_read()
        if input_raw is None:
            print("Error: Capture source not found or broken.")
            returns = [ERROR, ERROR, ERROR]
            push_network_table(table, returns)
            print(returns)
            table.putString("vision-status", "error")
            table.putString(enabled_key, "disabled")

            return 
        else:
            table.putString("vision-status", "")

        input_image = input_raw[0]
        timestamp = input_raw[1]

        # Find colored object / box it with a rectangle
        masked_image = find_colored_object(input_image, debug=DEBUG_MODE)

        object_rects = find_two_important_contours(masked_image, debug=DEBUG_MODE)
        object_rect, object_rect_2 = object_rects
        '''
        Uncomment below if you want to save images to output for debugging
        '''
        print(object_rects)
        cv2.imwrite("output/mask_%d.jpg"%frames, masked_image)
        cv2.imwrite("output/frame_%d.jpg"%frames,input_image) # Take out later to speed up
        for rectangle in object_rects:
            if rectangle is not -1:
                box_points = cv2.boxPoints(rectangle)
                box_points = np.int0(box_points)
                cv2.drawContours(input_image, [box_points], 0, (0, 255, 0), 2)
        cv2.imwrite("output/boxed_%d.jpg"%frames,input_image) # Take out later to speed up

        # DEBUG mode code
        if object_rect is -1 and object_rect_2 is -1:
            returns = [ERROR, ERROR, timestamp]
            push_network_table(table, returns)
            table.putString(enabled_key, "disabled")
            print(returns)
            if DEBUG_MODE:
                output_image = cv2.resize(input_image, (0, 0), fx=screen_resize, fy=screen_resize)
                cv2.imshow("Output", output_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # Quit if q key is pressed
                    return
            return 

        # if rectangles exist
        if not object_rect == -1 and not object_rect_2 == -1:
            # find_vert_angle finds the depth / angle of the object
            # find_hor_angle finds horizontal angle to object
            rect_x_midpoint, high_point = find_rectangle_highpoint(object_rects)
            vangle = find_angle(input_image, high_point, vert_focal_length) # vangle - 'V'ertical angle
            hangle = find_angle(input_image, rect_x_midpoint, hor_focal_length, vertical=False) # hangle - 'H'orizontal angle
            returns = [hangle, vangle, timestamp]
        else:
            returns = [ERROR, ERROR, timestamp]
            
        print(returns)
        # set and push network table
        push_network_table(table, returns)
        table.putString(enabled_key, "disabled")

        # Create output image to display in debug mode
        if DEBUG_MODE:
            output_image = draw_output_image(input_image, object_rects, 0, vangle, hangle, highpoint = (int(rect_x_midpoint), int(high_point)))
            if screen_resize != 1:
                output_image = cv2.resize(output_image, (0, 0), fx=screen_resize, fy=screen_resize)
            cv2.imshow("Output", output_image)

            # Check for key presses
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # Quit if q key is pressed
                return
            # Handle mouse clicks
            # Comment this out if unneeded for maximum efficiency
            # currently somewhat broken
            cv2.setMouseCallback("Output", mouse_click_handler, {"input_image": input_image, "screen_resize": screen_resize})
    except Exception as e:
        print(e)
           
# Classes
class ThreadedVideo:
    '''This class creates a thread for video capturing. self.stream is the capture stream'''
    def __init__(self, resize_value, src):
        '''
        Init class that initializes a video stream and a single
        video capture image.
        '''
        self.stream = cv2.VideoCapture(src)
        self.src = src
        self.width = 1000
        if self.opened():
            self.grabbed = (read_video_image(self.stream, resize_value), round(time.time()*1000))
        self.stopped = False
        self.frameCount = 0
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

    def raw_read(self):
        '''Returns the raw frame with the original video input's image size.'''
        if self.opened():
            self.grabbed = (read_video_image(self.stream), round(time.time()*1000))
        else:
            self.grabbed = None
        
        return self.grabbed

    def stream(self):
        '''Returns the OpenCV stream'''
        return self.stream

    def opened(self):
        '''Returns a boolean if the OpenCV stream is open'''
        cameraOpen = False;
        try:
            #Checks if a camera is connected if symlink is not broken - if it is throws error and caught below
            os.stat("/dev/video" + `self.src`)
            #Reopens stream so camera reconnects
            self.stream.open(self.src)

            #Sets exposure and other camera properties for camera
            os.system("v4l2-ctl -d /dev/video" + `self.src` + " -c exposure_auto=1 -c exposure_absolute=100 -c brightness=10 -c white_balance_temperature_auto=0 -c backlight_compensation=0 -c contrast=10 -c saturation=200")
            
            cameraOpen = True
        except OSError:
            #Releases a stream if its camera is disconnected
            print("Camera with source " + `self.src` + " is not connected!")
            self.stream.release()
            cameraOpen = False

        return cameraOpen

    def update(self):
        '''Grabs new video images from the current video stream.'''
        while not self.stopped:
            self.opened()

# Methods
def push_network_table(table, return_list):
    '''
    Pushes a tuple to the network table
    prints the table in Debug mode
    '''
    table.putNumberArray(NETWORK_KEY, return_list)
    if DEBUG_MODE:
        print(return_list)

def get_resize_values(capture, width=1920):
    '''
    Returns the fx / fy resize values needed to get the correct size image.
    Entirely aesthetic for debug mode.
    '''
    main_image = capture.read()[1]
    main_image_width = main_image.shape[1]
    return width / main_image_width


def read_video_image(capture, scale=1):
    '''
    Takes in a video capture source and a scale value (for resizing), and
    outputs the current frame as an image scaled by the scale value.
    Does some blurring to make the image easier to use.
    '''
    main_image = capture.read()[1]
    #if main_image is not None:
    #    main_image = cv2.resize(main_image, (0, 0), fx=scale, fy=scale)
    return main_image


def find_colored_object(image, debug=False):
    '''
    Takes in an image
    Outputs a black / white image with only the desired color object as white.
    '''
    # Find the part of the image with the specified color
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    masked_image = cv2.inRange(hsv_image, np.array(min_hsv), np.array(max_hsv))
    if debug:
        cv2.imshow("Color Mask", masked_image)
    return masked_image

def find_two_important_contours(image, debug=False):
    '''
    Finds the largest two contours in the inputted image (the tape strips).
    Returns the minimum area rectangle of each contour.
    If no contours are found, returns -1.
    '''
    # Blurs the image for better contour detection accuracy
    blur_image = cv2.medianBlur(image, 11)
    blur_image = cv2.GaussianBlur(blur_image, (1,1), 100)
    # blur_image = image.copy()
    center_x = image.shape[:2][1] / 2

    # Finds ALL the contours of the image
    # Note: the tree and chain things could probably be optimized better.
    contours = cv2.findContours(blur_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[0] #OpenCV 4 uses 1st element
    if len(contours) != 0:
        # Find the biggest area contour
        conarea = []
        dists = []
        ac_contours = []
        for i in range(len(contours)):
            ix = contours[i]
            area = cv2.contourArea(ix)
            if area > MIN_AREA:
                conarea.append(area)
                ac_contours.append(ix)
                moment = cv2.moments(ix)
                con_x = int(moment["m10"] / moment["m00"])
                distance = abs(center_x-con_x)
                dists.append(distance)

        try:
            centermost_dist = min(dists)
        except Exception:
            return [-1, -1]

        centermost_con = -1
        center_ind = -1
        if (centermost_dist != -1):
            center_ind = dists.index(centermost_dist)
            centermost_con = ac_contours[center_ind]
        if center_ind != -1:
           del ac_contours[center_ind]
           del dists[center_ind]

        isLeft = -1
        rect = -1
        rec2 = -1
        if centermost_con is not -1:
            rect = cv2.minAreaRect(centermost_con)

        rect_angle = rect[2]
        if (abs(rect_angle + 75) < 15):
            isLeft = True
        elif (abs(rect_angle + 15) < 15):
            isLeft = False

        sec_center_con = -1
        center_ind = -1

        try:
            sec_center_dist = min(dists)
        except Exception:
            sec_center_dist = -1

        if (sec_center_dist != -1):
            center_ind = dists.index(sec_center_dist)
            sec_center_con = ac_contours[center_ind]
        if center_ind != -1:
            del ac_contours[center_ind]
            del dists[center_ind]

        if sec_center_con is not -1:
            rec2 = cv2.minAreaRect(sec_center_con)

        if isLeft:
            while(rec2[0][0] <= rect[0][0]):
                sec_center_dist = min(dists)
                if (sec_center_dist != -1):
                    center_ind = dists.index(sec_center_dist)
                    sec_center_con = ac_contours[center_ind]
                if center_ind != -1:
                    del ac_contours[center_ind]
                    del dists[center_ind]
                if sec_center_con is not -1:
                    rec2 = cv2.minAreaRect(sec_center_con)
        elif not isLeft:
            while(rec2[0][0] >= rect[0][0]):
                sec_center_dist = min(dists)
                if (sec_center_dist != -1):
                    center_ind = dists.index(sec_center_dist)
                    sec_center_con = ac_contours[center_ind]
                if center_ind != -1:
                    del ac_contours[center_ind]
                    del dists[center_ind]
                if sec_center_con is not -1:
                    rec2 = cv2.minAreaRect(sec_center_con)
        else:
            rect2 = rect

        if debug:
            draw_image = image.copy()
            cv2.drawContours(draw_image, contours, -1, (255, 0, 0), 2)
            if rect is not -1:
                box_points = cv2.boxPoints(rect)
                box_points = np.int0(box_points)
                cv2.drawContours(draw_image, [box_points], 0, (0, 255, 0), 2)
            if rec2 is not -1:
                box_2p=cv2.boxPoints(rec2)
                box_2p = np.int0(box_2p)
                cv2.drawContours(draw_image, [box_2p], 0, (255, 0, 0), 2)
        return [rect, rec2]
    else:
        return [-1, -1]


def find_rectangle_highpoint(rectangles):
    '''
    Returns a tuple with the x midpoint and y highpoint.
    Highpoint is the average of the tallest point of each rectangle.
    '''
    # Find x midpoint and tallest y point of given rectangles
    highest_y = 9999999
    mid_x = 0
    for rectangle in rectangles:
        box_points = cv2.boxPoints(rectangle)
        mid_x += (box_points[0][0] + box_points[2][0]) / 2
        for box_point in box_points[1:]:
            if box_point[1] < highest_y:
                highest_y = box_point[1]
    mid_x /= len(rectangles)

    return (mid_x, highest_y)


def find_vert_focal_length(image, vert_fov):
    height = image.shape[:2][0]
    calc_focal_length = height / (2 * math.tan(math.radians(vert_fov / 2)))
    return calc_focal_length

def find_hor_focal_length(image, hor_fov):
    width = image.shape[:2][1]
    calc_focal_length = width / (2 * math.tan(math.radians(hor_fov / 2)))
    return calc_focal_length


def find_angle(image, coord, focal_length, vertical=True):
    '''
    Returns the vertical/horizontal angle of the given y/x point in an image.
    This is the angle that the robot needs to look up / down in order to
    directly face the image. Requires the actual image and focal
    length of the camera.
    '''

    # Find center y point
    angle = ERROR
    if vertical:
        image_height = image.shape[:2][0]

        # Find center x point
        center_y = image_height / 2
        y_from_bottom = image_height - coord

        # Calculate the angle using fancy formula
        angle = -1 * math.degrees(math.atan((y_from_bottom - center_y) / focal_length))
    else:
        image_width = image.shape[:2][1]

        # Find center x point
        center_x = image_width / 2
        x_from_bottom = image_width - coord

        # Calculate the angle using fancy formula
        angle = -1 * math.degrees(math.atan((x_from_bottom - center_x) / focal_length))

    return angle

# Debug mode methods
def mouse_click_handler(event, x, y, flags, params):
    '''
    If the mouse is clicked, return the hsv values of that point.
    Useful for figuring out precise hsv ranges / troubleshooting detection.
    DEBUG function
    '''
    if event == cv2.EVENT_LBUTTONDOWN:
        try:
            hsv_image = cv2.cvtColor(params["input_image"], cv2.COLOR_BGR2HSV)
            screen_resize = params["screen_resize"]
        except NameError:
            print("Input image not found!")
        norm_x, norm_y = round(x/screen_resize), round(y/screen_resize)
        h, s, v = hsv_image[norm_y][norm_x]
        print("HSV value of point ({}, {}) is ({}, {}, {})".format(norm_x, norm_y, h, s, v))


def draw_output_image(image, rectanglelist, depth, vangle, hangle, highpoint=None): ##edited
    '''
    Draws everything on the original image, and returns an image with
    all the information found by this program.
    DEBUG function
    '''
    output_image = image.copy()
    # Get width and height of the image
    height, width = output_image.shape[:2]
    # Draw the rectangle that boxes in the object
    for rectangle in rectanglelist:
        box_points = cv2.boxPoints(rectangle)
        box_points = np.int0(box_points)
        cv2.drawContours(output_image, [box_points], 0, (0, 255, 0), 2)
    # Draw the depth / angle of the object
    cv2.putText(output_image, "%.2f inches" % depth, (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
    cv2.putText(output_image, "%.2f degrees v" % vangle, (10, height - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    cv2.putText(output_image, "%.2f degrees h" % hangle, (10, height - 110),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    if highpoint is not None:
        cv2.circle(output_image, highpoint, 10, (0, 255, 255), thickness=10)
    return output_image


# causes program to run main method when program is run, but allows modular import allows
if __name__ == "__main__":
    main()



