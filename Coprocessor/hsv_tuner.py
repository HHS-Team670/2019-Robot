import cv2
import numpy as np
import math


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
    main_image = cv2.imread("test.jpg")
    main_image = cv2.resize(main_image, (0, 0), fx=scale, fy=scale)
    return main_image

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


def draw_output_image(image, rectangle, depth, angle):
    '''
    Draws everything on the original image, and returns an image with
    all the information found by this program.
    '''
    output_image = image.copy()
    # Get width and height of the image
    height, width = output_image.shape[:2]
    # Draw the rectangle that boxes in the object
    box_points = cv2.boxPoints(rectangle)
    box_points = np.int0(box_points)
    cv2.drawContours(output_image, [box_points], 0, (0, 255, 0), 2)
    # Draw the depth / angle of the object
    cv2.putText(output_image, "%.2f ft" % depth, (10, height - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 0, 255), 3)
    cv2.putText(output_image, "%.2f degrees" % angle, (10, height - 60),
                cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 3)
    return output_image



image_width = 1080  # Desired width of inputted image (for processing speed)
screen_resize = 1  # Scale that the GUI image should be scaled to

# Video capture / resizing stuff
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

    if object_rect == -1:
        # print("No contours found. Assuming no colored object was found.")
        output_image = cv2.resize(input_image, (0, 0), fx=screen_resize, fy=screen_resize)
        cv2.imshow("Output", output_image)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            # Quit if q key is pressed
            break
        continue

    if screen_resize != 1:
        output_image = cv2.resize(output_image, (0, 0), fx=screen_resize, fy=screen_resize)
    cv2.imshow("Output", output_image)

    # Check for key presses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # Quit if q key is pressed
        break

    # Handle mouse clicks
    # Comment this out if unneeded for maximum efficiency
    cv2.setMouseCallback("Output", mouse_click_handler)

# Release & close when done
capture.release()
cv2.destroyAllWindows()