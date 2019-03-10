import cv2

def mouse_click_handler(event, x, y, flags, params):
    '''
    If the mouse is clicked, return the hsv values of that point.
    Useful for figuring out precise hsv ranges / troubleshooting detection.
    '''
    if event == cv2.EVENT_LBUTTONDOWN:
        hsv_image = cv2.cvtColor(input_image, cv2.COLOR_BGR2HSV)
        norm_x, norm_y = round(x), round(y)
        h, s, v = hsv_image[norm_y][norm_x]
        print("HSV value of point ({}, {}) is ({}, {}, {})".format(norm_x, norm_y, h, s, v))
        global image
        cv2.putText(input_image, "HSV value of point ({}, {}) is ({}, {}, {})".format(norm_x, norm_y, h, s, v), (10, 535), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

# Read input image from file
input_image = cv2.imread("test.jpg") #relative or absolute file path
while True:
    cv2.imshow("HSV", input_image)

    # Check for key presses
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        # Quit if q key is pressed
        break
    
    # Handle mouse clicks
    cv2.setMouseCallback("HSV", mouse_click_handler)

# Close when done
cv2.destroyAllWindows()