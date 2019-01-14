'''
Kyle Fu || Rishab Borah || Navaneet Kadaba || Eshan Jain || Harinandan K

Usage notes:
You can calibrate the focal distance or object diagonal value by pressing
the key 'c' while holding a detected colored object a known distance
away from the camera. You need either a known diagonal length of the object
or the known focal length of the camera. The bigger the object, the more
accurate this calibration will be (as long as it fits into the camera frame).
Check the terminal output to choose whether to calibrate the focal length or
diagonal.
You can click on a point on the output image to print out the hsv value of
that point. Useful for finding specific hsv color ranges.
'''

import copy
import math
from threading import Thread
import cv2
import imutils
import numpy as np
import datetime

DEBUG_MODE = True # NOTE MAKE @FALSE TO MAKE NO SCREENS APPEAR
ERROR = -99999
# Variables (These should be changed to reflect the camera)
capture_source = 0  # Number of port for camera, file path for video
capture_color = 'g'  # Possible: r (Red), g (Green), b (Blue), y (Yellow), x (reflector tape)
known_object_height = 12.75  # Height of the tape from the ground (in inches)
known_camera_height = 2.0
camera_fov_vertical = 39.7  # FOV of the camera (in degrees)
camera_fov_horizontal = 60.0
image_width = 1080  # Desired width of inputted image (for processing speed)
screen_resize = 1  # Scale that the GUI image should be scaled to
calibrate_angle = 0  # Test to calibrate the angle and see if that works
exposure = -8
timestamp = datetime.datetime.now()
# HSV Values to detect
min_hsv = [58, 0, 254]
max_hsv = [67, 62, 255]

# Min area
MIN_AREA = 2000

returns = [-99999, -99999, 0]

def main():
    '''
    Main.
    TODO Separate output images into a debug mode, if debug mode enabled
    show vid and prints, else no
    TODO Adjust depth
    '''
    # Video capture / resizing stuff
    cv2.VideoCapture(capture_source).set(cv2.CAP_PROP_EXPOSURE, exposure)
    vs = ThreadedVideo(screen_resize, capture_source).start()
    # resize_value = get_resize_values(vs.stream, image_width)

    # This may not need to be calculated, can use Andra's precalculated values
    vert_focal_length = find_vert_focal_length(vs.raw_read(),
                                               camera_fov_vertical)
    hor_focal_length = find_hor_focal_length(vs.raw_read(),
                                             camera_fov_horizontal)

    while True:
        # Read input image from video
        input_image = vs.raw_read()
        timestamp = datetime.datetime.now()
        if input_image is None:
            print("Error: Capture source not found or broken.")
            returns = [ERROR, ERROR, timestamp]
            push_network_table(returns)
            if DEBUG_MODE:
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # Quit if q key is pressed
                    break
            continue
        # Find colored object / box it with a rectangle
        masked_image = find_colored_object(input_image, capture_color,
                                           debug=DEBUG_MODE)
        # Find the two biggest colored objects (two pieces of tape)
        # TODO change this to filter rectangles for 2 centermost in screen
        # TODO check if invalid rectangles return -99999
        # TODO if both or one center rectangles: continue as normal

        object_rects = find_largest_contour(masked_image, debug=DEBUG_MODE)
        object_rect, object_rect_2 = object_rects

        # DEBUG mode code
        if object_rect is -1 and object_rect_2 is -1:
            returns = [ERROR, ERROR, timestamp]
            push_network_table(returns)
            if DEBUG_MODE:
                output_image = cv2.resize(input_image, (0, 0), fx=screen_resize,
                                        fy=screen_resize)
                cv2.imshow("Output", output_image)
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    # Quit if q key is pressed
                    break
            continue
        if object_rect == -1:
            del object_rect
            del object_rects[0]
        elif object_rect_2 == -1:
            del object_rect_2
            del object_rects[1]
        '''
        elif (object_rect == -1) != (object_rect_2 == -1):
            output_image = cv2.resize(input_image, (0, 0), fx=screen_resize,
                                      fy=screen_resize)
            cv2.imshow("Output", output_image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                # Quit if q key is pressed
                break
            continue
        '''

        # Find the depth / angle of the object
        # find_horizontal_angle finds horizontal angle to object if needed
        # angle = find_vertical_angle(input_image, object_rects, camera_fov)
        # angle -= calibrate_angle
        rect_x_midpoint, high_point = find_rectangle_highpoint(object_rects)
        vangle = find_vert_angle(input_image, high_point, vert_focal_length)
        hangle = find_hor_angle(input_image, rect_x_midpoint, hor_focal_length)
        depth = depth_from_angle(input_image, object_rects, vangle, hangle,
                                 known_object_height - known_camera_height)
        adjusted_depth = adjust_depth(depth, hangle)
        # adjusted_depth = depth

        returns = [hangle, adjusted_depth, timestamp]
        push_network_table(returns)
        # Create output image to display
        if DEBUG_MODE:
            output_image = draw_output_image(input_image,
                                            object_rects,
                                            adjusted_depth, 
                                            vangle, 
                                            hangle, 
                                            hipoint = (int(rect_x_midpoint), int(high_point)))
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

    # Release & close when done
    vs.stop()
    vs.stream.release()
    cv2.destroyAllWindows()


class ThreadedVideo:
    '''This class creates a thread for video capturing.'''
    def __init__(self, resize_value, src):
        '''
        Init class that initializes a video stream and a single
        video capture image.
        '''
        self.stream = cv2.VideoCapture(src)
        self.width = 1000
        self.grabbed = read_video_image(self.stream, resize_value)
        self.frame = imutils.resize(self.grabbed, width=self.width)
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
        '''Returns the resized frame.'''
        return self.frame

    def raw_read(self):
        '''Returns the raw frame with the original video input's image size.'''
        return self.grabbed

    def update(self):
        '''Grabs new video images from the current video stream.'''
        while not self.stopped:
            self.grabbed = read_video_image(self.stream, self.resize)
            self.frame = imutils.resize(self.grabbed, width=self.width)

def push_network_table(return_list):
    '''placeholder function for network table pushing--prints the list'''
    if DEBUG_MODE:
        print(return_list)

def get_resize_values(capture, width=1920):
    '''
    Returns the fx / fy resize values needed to get the correct size image.
    Mainly for optimization purposes. Entirely aesthetic for debug mode.
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


# TODO: Make this take in HSV values from the start and not futz around
# with converting rgb -> hsv
def find_colored_object(image, capture_color='y', debug=False):
    '''
    Takes in an image and the capture color (r, g, b, or y).
    Outputs a black / white image with only the desired color object as white.
    '''
    # Find the part of the image with the specified color

    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if capture_color == 'y':
        masked_image = cv2.inRange(hsv_image, np.array([17, 125, 125]),
                                   np.array([36, 255, 255]))  # Yellow
    elif capture_color == 'b':
        masked_image = cv2.inRange(hsv_image, np.array([95, 50, 50]),
                                   np.array([125, 255, 255]))  # Blue
    elif capture_color == 'g':
        masked_image = cv2.inRange(hsv_image, np.array([35, 50, 50]),
                                   np.array([80, 255, 255]))  # Green
        # Correct values: [58, 0, 254], [67, 62, 255]
    elif capture_color == 'x':
        # TODO this will eventually be the only check
        masked_image = cv2.inRange(hsv_image, np.array(min_hsv), np.array(max_hsv))
    else:
        mask_1 = cv2.inRange(hsv_image, np.array([0, 50, 50]),
                             np.array([5, 255, 255]))
        mask_2 = cv2.inRange(hsv_image, np.array([165, 50, 50]),
                             np.array([180, 255, 255]))
        masked_image = cv2.bitwise_or(mask_1, mask_2)  # Red red red red
    if debug:
        cv2.imshow("Color Mask", masked_image)
    return masked_image


def find_largest_contour(image, debug=False):
    '''
    Finds the largest two contours in the inputted image (the tape strips).
    Returns the minimum area rectangle of each contour.
    If no contours are found, returns -1.
    '''
    # Blurs the image for better contour detection accuracy
    #blur_image = cv2.medianBlur(image, 5)
    #blur_image = cv2.GaussianBlur(blur_image, (5, 5), 0)
    blur_image = image.copy()
    center_x = image.shape[:2][1] / 2

    # Finds ALL the contours of the image
    # Note: the tree and chain things could probably be optimized better.
    contours = cv2.findContours(blur_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[1]
    if len(contours) != 0:
        # Find the biggest area contour
        conarea = []
        dists = []
        ac_contours = []
        for i in range(len(contours)):
            ix = contours[i]
            area = cv2.contourArea(ix)
            if area > 100:
                conarea.append(area)
                ac_contours.append(ix)
                moment = cv2.moments(ix)
                con_x = int(moment["m10"] / moment["m00"])
                distance = abs(center_x-con_x)
                dists.append(distance)

        #biggest_contour = max(contours, key=cv2.contourArea)
        centermost_dist = min(dists, default=-1)
        centermost_con = -1
        center_ind = -1
        if (centermost_dist != -1):
            center_ind = dists.index(centermost_dist)
            centermost_con = ac_contours[center_ind]
        if center_ind != -1:
           del ac_contours[center_ind]
           del dists[center_ind]

        sec_center_con = -1
        center_ind = -1
        sec_center_dist = min(dists, default=-1)
        if (sec_center_dist != -1):
            center_ind = dists.index(sec_center_dist)
            sec_center_con = ac_contours[center_ind]
        if center_ind != -1:
            del ac_contours[center_ind]
            del dists[center_ind]
        
        rect = -1
        rec2 = -1
        if centermost_con is not -1:
            rect = cv2.minAreaRect(centermost_con)
        if sec_center_con is not -1:
            rec2 = cv2.minAreaRect(sec_center_con)
        '''
        contours2 = copy.deepcopy(contours)
        biggest_contour=max(conarea)
        del contours2[conarea.index(biggest_contour)]
        biggest_contour=contours[conarea.index(biggest_contour)]
        if len(contours) != 1:
            area2 = []
            for i in contours2:
                area2.append(cv2.contourArea(i))
            sec_big_con = max(area2)
            sec_big_con = contours2[area2.index(sec_big_con)]
            rec2 = cv2.minAreaRect(sec_big_con)
        else:
            rec2 = cv2.minAreaRect(biggest_contour)
        # Creating a rotated minimum area rectangle
        rect = cv2.minAreaRect(biggest_contour)
        '''
        if debug:
            cv2.imshow("Blurred", blur_image)
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
            cv2.imshow("Contours / Rectangle", image)
        return [rect, rec2]
    else:
        return [-1, -1]


def depth_from_angle(image, rectangles, vangle, hangle, known_height):
    '''
    Returns the depth of the tape when given the height of the tape (from the
    center of the camera to the middle of the tape), the angle to the center
    of the tape, the focal length, and the original image (to get its size).
    '''
    # Keep tangent from being undefined
    if vangle == 0:
        vangle = 0.0001
    # Do some calculations

    depth = known_height / math.tan(abs(vangle))
    # This was an adjustment to adjust depth for an object that is offcenter
    # adjusted_depth = depth / math.cos(abs(hangle))
    return depth


def find_vert_focal_length(image, vert_fov):
    '''
    Calculates the vertical focal length when given
    the image and vertical FOV.
    '''
    # Find image height
    height = image.shape[:2][0]

    calc_focal_length = height / (2 * math.tan(math.radians(vert_fov / 2)))
    return calc_focal_length


def find_hor_focal_length(image, hor_fov):
    width = image.shape[:2][1]

    calc_focal_length = width / (2 * math.tan(math.radians(hor_fov / 2)))
    return calc_focal_length


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

def find_vert_angle(image, y, focal_length):
    '''
    Returns the vertical angle of the given y point in an image.
    This is the angle that the robot needs to look up / down in order to
    directly face the image. Requires the actual image and focal
    length of the camera.
    '''

    # Find center y point
    image_height = image.shape[:2][0]
    center_y = image_height / 2
    y_from_bottom = image_height-y

    # Calculate the angle using fancy formula
    vertical_angle = -1 * math.degrees(math.atan((y_from_bottom - center_y) / focal_length))
    return vertical_angle

def find_hor_angle(image, x, focal_length):
    '''
    Returns the horizontal angle of the given x point in an image.
    This is the angle that the robot needs to look left / right in order to
    directly face the image. Requires the actual image and focal
    length of the camera.
    '''

    # Find center x point
    image_width = image.shape[:2][1]
    center_x = image_width / 2
    x_from_bottom = image_width-x

    # Calculate the angle using fancy formula
    horizontal_angle = -1 * math.degrees(math.atan((x_from_bottom - center_x) / focal_length))
    return horizontal_angle

def adjust_depth(depth, angle):
    '''
    Adjusts the depth based on the angle to the object (using simple trig).
    Returns the adjusted depth.
    '''
    cos_angle = math.cos(math.radians(abs(angle)))
    if cos_angle != 0:
        adjusted_depth = depth / cos_angle
    else:
        adjusted_depth = 0
    return adjusted_depth


def mouse_click_handler(event, x, y, flags, params):
    '''
    If the mouse is clicked, return the hsv values of that point.
    Useful for figuring out precise hsv ranges / troubleshooting detection.
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


def draw_output_image(image, rectanglelist, depth, vangle, hangle, hipoint=None): ##edited
    '''
    Draws everything on the original image, and returns an image with
    all the information found by this program.
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
    if hipoint is not None:
        cv2.circle(output_image, hipoint, 10, (0, 255, 255), thickness=10)
    return output_image


if __name__ == "__main__":
    main()
