# Kyle Fu & Rishab Borah
# OpenCV - Mono Depth / Angle Detection

# Capabilities (Uses one camera):
# Finding accurate distance to a colored object centered in the frame (~3 inch
# error margin, up to 16 feet away)
# Finding the angle to the center of the colored object relative to the camera
# Displaying an output image with the previously calculated info

# Issues:
# Isn't accurate for an object that is not directly facing the camera
# Fix - Don't know yet. This one's complicated. I'll get to it after finals.
# Only works on mac if contours[1] is changed to contours[0] in the
# find_largest_contour() function
# Fix - Detect if contours[0] or contours[1] has the image array data,
# and choose the correct one based on that (probably easy to do, DO THIS ME)
# Focal distance inputted into this program might not be the
# exact focal distance of the camera
# Fix - I can't test if this is an issue (although I assume it is),
# so I don't know how to fix it.
# Some efficiency / optimization things probably exist in this code
# Fix - In the final version of this, remove the click handling and the
# calibration code (or at least optimize it). Also, find ways to record some
# constant / rarely changing values to avoid calculating them every frame.

# Usage notes:
# You can calibrate the focal distance or object diagonal value by pressing
# the key 'c' while holding a detected colored object a known distance
# away from the camera. You need either a known diagonal length of the object
# or the known focal length of the camera. The bigger the object, the more
# accurate this calibration will be (as long as it fits into the camera frame).
# Check the terminal output to choose whether to calibrate the focal length or
# diagonal.
# You can click on a point on the output image to print out the hsv value of
# that point. Useful for finding specific hsv color ranges.

import cv2
import numpy as np
import math
import copy


def get_resize_values(capture, width=1920):
    '''
    Returns the fx / fy resize values needed to get the correct size image.
    Mainly for optimization purposes.
    '''
    main_image = capture.read()[1]
    image_width = main_image.shape[1]
    return width / image_width


def read_video_image(capture, scale=1):
    '''
    Takes in a video capture source and a scale value (for resizing), and
    outputs the current frame as an image scaled by the scale value.
    Does some blurring to make the image easier to use.
    '''
    main_image = capture.read()[1]
    main_image = cv2.resize(main_image, (0, 0), fx=scale, fy=scale)
    return main_image


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
        masked_image = cv2.inRange(hsv_image, np.array([45, 50, 50]),
                                   np.array([80, 255, 255]))  # Green
    else:
        mask_1 = cv2.inRange(hsv_image, np.array([0, 50, 50]),
                             np.array([5, 255, 255]))
        mask_2 = cv2.inRange(hsv_image, np.array([165, 50, 50]),
                             np.array([180, 255, 255]))
        masked_image = cv2.bitwise_or(mask_1, mask_2)  # Red red red red
    if debug:
        cv2.imshow("Color Mask", masked_image)
    return masked_image


def find_largest_contour(image, debug=False): ##edited
    '''
    Finds the largest contour in the inputted image.
    Returns the minimum area rectangle of that contour.
    If no contours are found, returns -1.
    '''
    # Blurs the image for better contour detection accuracy
    #blur_image = cv2.medianBlur(image, 5)
    #blur_image = cv2.GaussianBlur(blur_image, (5, 5), 0)
    blur_image = image.copy()

    # Finds ALL the contours of the image
    # Note: the tree and chain things could probably be optimized better.
    contours = cv2.findContours(blur_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = contours[1]
    if len(contours) != 0:
        # Find the biggest area contour
        conarea = []
        for i in contours:
            conarea.append(cv2.contourArea(i))
        ##biggest_contour = max(contours, key=cv2.contourArea)
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
        
        if debug:
            cv2.imshow("Blurred", blur_image)
            draw_image = image.copy()
            cv2.drawContours(draw_image, contours, -1, (255, 0, 0), 2)
            cv2.drawContours(draw_image, contours2, -1, (0, 255, 0), 2)
            box_points = cv2.boxPoints(rect)
            box_2p=cv2.boxPoints(rec2)
            box_points = np.int0(box_points)
            box_2p = np.int0(box_2p)
            cv2.drawContours(draw_image, [box_points], 0, (0, 255, 0), 2)
            cv2.drawContours(draw_image, [box_2p], 0, (255, 0, 0), 2)
            cv2.imshow("Contours / Rectangle", image)
        return [rect, rec2]
    else:
        return [-1, -1]


def find_depth(image, rectangle, focal_length, known_diagonal):
    '''
    Returns the depth of the given rectangle in an image (in feet). Needs the
    original image (to get its size), the rectangle itself, the focal length
    of the camera (in ???), and the length of the object's diagonal (in feet).
    '''
    # Pythagorean theorem
    width, height = rectangle[1]
    diagonal = (width ** 2 + height ** 2) ** 0.5
    # Normalize the diagonal length for different-sized images
    diagonal /= image.shape[:2][1]
    # Find the depth
    if diagonal != 0:
        depth = known_diagonal * focal_length / diagonal
        return depth
    else:
        return 0


def find_angle(image, rectangle, fov):
    '''
    Returns the angle of the given rectangle in an image. This is the angle
    that the robot needs to turn in order to directly face the image.
    Requires the FOV of the camera.
    '''
    # Find image width
    width = image.shape[:2][1]
    # Find midpoint of given rectangle
    box_points = cv2.boxPoints(rectangle)
    mid_x = (box_points[0][0] + box_points[2][0]) / 2
    # Calculate angle based on image width, fov, and rectangle midpoint
    angle = (mid_x / width - 0.5) * fov
    return angle


def adjust_depth(depth, angle):
    '''
    Adjusts the depth based on the angle to the object (using simple trig).
    Returns the adjusted depth.
    '''
    sin_angle = math.sin(math.radians(90 - abs(angle)))
    if sin_angle != 0:
        adjusted_depth = depth / sin_angle
    else:
        adjusted_depth = 0
    return adjusted_depth


def calibrate_camera(image, rectangle, calibrate_distance, focal_length=-1, known_diagonal=-1):
    '''
    Returns the correct focal length of the camera with the given object,
    or the diagonal of the current object depending on whether the current
    focal length or object's diagonal length is inputted. Works based on a
    known distance of the object from the camera (only 1 unknown left).
    Assumes that the object is centered in the image.
    '''
    # Pythagorean theorem
    width, height = rectangle[1]
    diagonal = (width ** 2 + height ** 2) ** 0.5
    # Normalize the diagonal length for different-sized images
    diagonal /= image.shape[:2][1]
    if known_diagonal != -1:
        correct_focal_length = calibrate_distance * diagonal / known_diagonal
        return correct_focal_length
    elif focal_length != -1:
        correct_diagonal = calibrate_distance * diagonal / focal_length
        return correct_diagonal
    print("Someone forgot their arguments! :)")
    return -1


def mouse_click_handler(event, x, y, flags, params):
    '''
    If the mouse is clicked, return the hsv values of that point.
    Useful for figuring out precise hsv ranges / troubleshooting detection.
    '''
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
        norm_x, norm_y = round(x/screen_resize), round(y/screen_resize)
        h, s, v = hsv_image[norm_y][norm_x]
        print("HSV value of point ({}, {}) is ({}, {}, {})".format(norm_x, norm_y, h, s, v))


def draw_output_image(image, rectanglelist, depth, angle): ##edited
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
    cv2.putText(output_image, "%.2f ft" % depth, (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
    cv2.putText(output_image, "%.2f degrees" % angle, (10, height - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    return output_image


# Variables (These should be changed to reflect the camera)
capture_source = 0  # 0 for camera, file path for video
capture_color = 'g'  # Possible: r (Red), g (Green), b (Blue), y (Yellow)
known_object_diagonal = 1.5  # Length of the diagonal of the object (in feet)
focal_length = 0.71  # Focal length of the camera (in inches)
camera_fov = 50  # FOV of the camera (in degrees)
image_width = 1080  # Desired width of inputted image (for processing speed)
screen_resize = 1  # Scale that the GUI image should be scaled to

# Video capture / resizing stuff
capture = cv2.VideoCapture(capture_source)
resize_value = get_resize_values(capture, image_width)
while True:
    # Read input image from video
    input_image = read_video_image(capture, resize_value)
    if input_image is None:
        print("Error: Capture source not found or broken.")
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Quit if q key is pressed
            break
        continue

    # Find colored object / box it with a rectangle
    masked_image = find_colored_object(input_image, capture_color, debug=False)
    object_rect = find_largest_contour(masked_image, debug=False)
    obj_r_2 = object_rect[1]
    object_rect=object_rect[0]
    if object_rect == -1:
        # print("No contours found. Assuming no colored object was found.")
        output_image = cv2.resize(input_image, (0, 0), fx=screen_resize, fy=screen_resize)
        cv2.imshow("Output", output_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Quit if q key is pressed
            break
        continue

    # Find the depth / angle of the object
    angle = find_angle(input_image, object_rect, camera_fov)
    depth = find_depth(input_image, object_rect, focal_length, known_object_diagonal)
    adjusted_depth = adjust_depth(depth, angle)

    # Create output image to display
    output_image = draw_output_image(input_image, [object_rect, obj_r_2], adjusted_depth, angle)
    ##output2=draw_output_image(input_image, obj_r_2, adjusted_depth, angle)
    # Some debug text that was used to see if the adjusted distance was working
    # cv2.putText(output_image, "%.2f ft (not adjusted)" % depth, (10, 535),
    #            cv2.FONT_HERSHEY_SIMPLEX, 1+0.5, (0, 0, 255), 2+2)
    # cv2.putText(output_image, "%.2f ft difference" % (adjusted_depth - depth), (10, 485),
    #            cv2.FONT_HERSHEY_SIMPLEX, 1+0.5, (0, 0, 255), 2+2)
    if screen_resize != 1:
        output_image = cv2.resize(output_image, (0, 0), fx=screen_resize, fy=screen_resize)
        ##output2 = cv2.resize(output_2, (0, 0), fx=screen_resize, fy=screen_resize)
    cv2.imshow("Output", output_image)
    ##cv2.imshow("out2", output2)

    # Check for key presses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # Quit if q key is pressed
        break
    elif key == ord('c'):
        # Calibrate if c key is pressed
        # Currently calibrates focal length, but can easily be changed to
        # calibrate known diagonal instead
        # Comment this out if unneeded for maximum efficiency
        print("Make sure the object is CENTERED in the camera (should be ~0 degrees)! If it isn't, abort the calibration.")
        calibrate_distance = input("How far away is the detected object (in feet)? Type q to abort the calibration.\n-> ")
        try:
            calibrate_distance = float(calibrate_distance)
        except ValueError:
            print("Aborting calibration...")
            continue
        choice = input("Calibrate (f)ocal length or (d)iagonal? Type q to abort the calibration.\n-> ")
        if choice == 'f':
            focal_length = round(calibrate_camera(input_image, object_rect, calibrate_distance, known_diagonal=known_object_diagonal), 3)
            print("Calibrated! New focal length: {}".format(focal_length))
            print("Take this value down if needed: It will revert back after the program is stopped!")
        elif choice == 'd':
            known_object_diagonal = round(calibrate_camera(input_image, object_rect, calibrate_distance, focal_length=focal_length), 3)
            print("Calibrated! New known object diagonal: {}".format(known_object_diagonal))
            print("Take this value down if needed: It will revert back after the program is stopped!")
        else:
            print("Aborting calibration...")

    # Handle mouse clicks
    # Comment this out if unneeded for maximum efficiency
    cv2.setMouseCallback("Output", mouse_click_handler)

# Release & close when done
capture.release()
cv2.destroyAllWindows()
