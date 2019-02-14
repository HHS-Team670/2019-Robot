'''
Arjun Sampath || Kyle Fu || Rishab Borah || Navaneet Kadaba || Eshan Jain || Harinandan K
Usage notes:
Set DEBUG_MODE to True in order to make screens appear showing what robot sees
Depth detection is reliant on constants--the known object height, the known camera's height, the camera's FOV, and focal length
You can click on a point on the output image to print out the hsv value of
that point in debug mode. Useful for finding specific hsv color ranges.
Pushes a tuple (angle, distance, timestamp) to network tables--for Shaylan's robot auton drive code
copper goes to negative
'''

import copy
import logging
import math
from threading import Thread
import cv2
import imutils
import numpy as np
import time
from networktables import NetworkTables
import os

logging.basicConfig(level=logging.DEBUG)
logger = logging.getLogger('logger')
DEBUG_MODE = False # NOTE MAKE this @FALSE TO MAKE NO SCREENS APPEAR
ERROR = -99999

# TODO SET THESE to correct values
ROBORIO_IP = "10.6.70.2" # TODO set this to roborio ip of network table
NETWORK_TABLE_NAME = "SmartDashboard" # TODO set this to network table name
NETWORK_KEY = "reflect_tape_vision_data"

# Variables (These should be changed to reflect the camera)
front_capture_source = 0 # Number of USB port for front camera
back_capture_source = 1 # Number of USB port for back camera, file path for video
# target_height = 31  # Height of the tape from the ground (in inches) # Now taken off of network tables
known_camera_height = 8.75 
camera_fov_vertical = 39.7  # FOV of the camera (in degrees)
camera_fov_horizontal = 60.0
image_width = 1080  # Desired width of inputted image (for processing speed)
screen_resize = 1  # Scale that the GUI image should be scaled to
# calibrate_angle = 0  # Test to calibrate the angle and see if that works
timestamp = round(time.time() * 1000) # time in milliseconds
camera_vertical_angle = 0.0 # camera angle offset up or down (positive=up, negative=down)
camera_horizontal_angle = 0.0 # camera angle offset right left
# camera_horizontal_offset = 0.0 # camera horizontal offset distancewise in inches


# Keys for network table entries
camera_key = "vision-camera"

# HSV Values to detect
min_hsv = [50, 220, 80]
max_hsv = [70, 255, 210]

# Min area to make sure not to pick up noise
MIN_AREA = 100

# Network table (by default returns error codes, but changes in program)
returns = [ERROR, ERROR, timestamp]

def main():
    '''
    Main method, runs when program is run.
    '''
    # initialize network tables

    os.system("v4l2-ctl -d /dev/video0 -c exposure_auto=1 -c exposure_absolute=.01 -c brightness=0 -c white_balance_temperature_auto=0 -c backlight_compensation=0 -c contrast=10 -c saturation=200")
    os.system("rm -rf output")
    os.system("mkdir output")
    print("start")
    NetworkTables.initialize(server=ROBORIO_IP)
    print("init nt")
    table = NetworkTables.getTable(NETWORK_TABLE_NAME)

    print("get table")
    print(table)
    #Video capture / resizing stuff
    vs_front = ThreadedVideo(screen_resize, front_capture_source).start()
    vs_back = ThreadedVideo(screen_resize, back_capture_source).start()

    # resize_value = get_resize_values(vs.stream, image_width) # uncomment if screen resize is desired

    # This may not need to be calculated, can use Andra's precalculated values
    front_vert_focal_length = find_vert_focal_length(vs_front.raw_read()[0], camera_fov_vertical)
    front_hor_focal_length = find_hor_focal_length(vs_front.raw_read()[0], camera_fov_horizontal)
    back_vert_focal_length = find_vert_focal_length(vs_back.raw_read()[0], camera_fov_vertical)
    back_hor_focal_length = find_hor_focal_length(vs_back.raw_read()[0], camera_fov_horizontal)

    startTime = time.time()
    frameCount = 0
    while True:

        # gets which camera to use (front or back)
        camera = table.getEntry(camera_key)

        vs = None
        if camera is "front":
            vs = vs_front
        else:
            vs = vs_back

        try:
            print(frameCount/(time.time() - startTime))
            frameCount += 1
            # Read input image from video
            input_raw = vs.raw_read()
            input_image = input_raw[0]
            timestamp = input_raw[1]
            if input_image is None:
                print("Error: Capture source not found or broken.")
                returns = [ERROR, ERROR, timestamp]
                push_network_table(table, returns)

                #Debug mode code
                if DEBUG_MODE:
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        # Quit if q key is pressed
                        break
                continue
            # Find colored object / box it with a rectangle
            masked_image = find_colored_object(input_image, debug=DEBUG_MODE)

            object_rects = find_two_important_contours(masked_image, debug=DEBUG_MODE)
            object_rect, object_rect_2 = object_rects

            if frameCount % 10 == 0:
                cv2.imwrite("output/mask_%d.jpg"%frameCount, masked_image)
                cv2.imwrite("output/frame_%d.jpg"%frameCount,input_image) # Take out later to speed up
                for rectangle in object_rects:
                    box_points = cv2.boxPoints(rectangle)
                    box_points = np.int0(box_points)
                    cv2.drawContours(input_image, [box_points], 0, (0, 255, 0), 2)
                cv2.imwrite("output/boxed_%d.jpg"%frameCount,input_image) # Take out later to speed up

            # DEBUG mode code
            if object_rect is -1 and object_rect_2 is -1:
                returns = [ERROR, ERROR, timestamp]
                push_network_table(table, returns)
                if DEBUG_MODE:
                    output_image = cv2.resize(input_image, (0, 0), fx=screen_resize,
                                            fy=screen_resize)
                    cv2.imshow("Output", output_image)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        # Quit if q key is pressed
                        break
                continue

            # if rectangles exist
            if not object_rect == -1 and not object_rect_2 == -1:
                # find_vert_angle finds the depth / angle of the object
                # find_hor_angle finds horizontal angle to object
                rect_x_midpoint, high_point = find_rectangle_highpoint(object_rects)
                if camera is "front":
                    vangle = find_angle(input_image, high_point, front_vert_focal_length) # vangle - 'V'ertical angle
                    hangle = find_angle(input_image, rect_x_midpoint, front_hor_focal_length, vertical=False) # hangle - 'H'orizontal angle
                else:
                    vangle = find_angle(input_image, high_point, back_vert_focal_length) # vangle - 'V'ertical angle
                    hangle = find_angle(input_image, rect_x_midpoint, back_hor_focal_length, vertical=False) # hangle - 'H'orizontal angle
                returns = [hangle, vangle, timestamp]
            else:
              returns = [ERROR, ERROR, timestamp]
            
            # set and push network table
            push_network_table(table, returns)

            # Create output image to display in debug mode
            if DEBUG_MODE:
                output_image = draw_output_image(input_image,
                                                 object_rects,
                                                 0,
                                                 vangle,
                                                 hangle,
                                                 highpoint = (int(rect_x_midpoint), int(high_point)))
                if screen_resize != 1:
                    output_image = cv2.resize(output_image, (0, 0),
                                              fx=screen_resize, fy=screen_resize)
                cv2.imshow("Output", output_image)

                # Check for key presses
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # Quit if q key is pressed
                    break
                # Handle mouse clicks
                # Comment this out if unneeded for maximum efficiency
                # currently somewhat broken
                cv2.setMouseCallback("Output", mouse_click_handler,
                                    {"input_image": input_image,
                                    "screen_resize": screen_resize})
        except Exception as e:
            print(e)
           
    # Release & close when done
    vs.stop()
    vs.stream.release()
    cv2.destroyAllWindows()

# Classes
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

# Methods
def push_network_table(table, return_list):
    '''
    Pushes a tuple to the network table
    prints the table in Debug mode
    '''
    #push_tuple = tuple(return_list)
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
    if main_image is not None:
        main_image = cv2.resize(main_image, (0, 0), fx=scale, fy=scale)
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

        centermost_dist = min(dists)
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
        sec_center_dist = min(dists)
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


def find_angle(image, y, focal_length, vertical=True):
    '''
    Returns the vertical/horizontal angle of the given y/x point in an image.
    This is the angle that the robot needs to look up / down in order to
    directly face the image. Requires the actual image and focal
    length of the camera.
    '''

    # Find center y point
    image_height = 0
    _offset = 0 # if camera is tilted
    if vertical:
        image_height = image.shape[:2][0]
        _offset = camera_vertical_angle
    else:
        image_height = image.shape[:2][1]
        _offset = camera_horizontal_angle

    # Find center y point
#    image_height = image.shape[:2][0]
    center_y = image_height / 2
    y_from_bottom = image_height-y

    # Calculate the angle using fancy formula
    _angle = -1 * math.degrees(math.atan((y_from_bottom - center_y) / focal_length))
    _angle -= _offset
    return _angle

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
