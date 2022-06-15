#!/usr/bin/env python3

import cv2 as cv
import numpy as np
import sys
import rospy
from rospy.exceptions import ROSInterruptException
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


cv_image = None

def callback(data):
    global cv_image

    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8") #converting data file to bgr8 colour space
        hsv_frame = cv.cvtColor(frame,cv.COLOR_BGR2HSV) # converting to HSV frame to do thresholding operation
        # gray= cv.cvtColor(frame,cv.COLOR_BGR2GRAY)
        hsv=hsv_frame.copy()

        #Setting red colour limits in HSV space
        lower1 = np.array([0, 100, 20])
        upper1 = np.array([4, 255, 255])

        lower2 = np.array([170, 100, 20])
        upper2 = np.array([179, 255, 255])

        lower_mask = cv.inRange(hsv_frame, lower1, upper1)
        upper_mask = cv.inRange(hsv_frame, lower2, upper2)
 
        full_mask = lower_mask + upper_mask
 
        hsv = cv.bitwise_and(hsv, hsv, mask=full_mask)
        h,s,gray = cv.split(hsv)

   
        # thresh = cv.threshold(blurred, 60, 255, cv.THRESH_BINARY)[1]


        blurred = cv.GaussianBlur(gray, (5, 5), 0)  


        cnts = cv.findContours(blurred, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # #drawing contours and dot inside the contour 
        # i=1
        for c in cnts:
            
            try:
                M = cv.moments(c)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # draw the contour and center of the shape on the image
                if cv.contourArea(c) > 50:
                    cv.drawContours(frame, [c], -1, (0, 255, 0), thickness=1)
                    cv.circle(frame, (cX, cY), 1, (255, 255, 255), -1)
                    cv.putText(frame, "object", (cX - 20, cY - 20),
                    cv.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)
                    print(len(c))
            except ZeroDivisionError:
                pass

            
        #  finding area to sort out the first and last object 




        cv.imshow("camera",frame)
        cv.waitKey(1)
    except CvBridgeError as e:
        print(e)

def main(args):
    rospy.init_node('tomato_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    image_sub = rospy.Subscriber("camera/color/image_raw2", Image, callback)
    try:
        rospy.spin()
    except ROSInterruptException:
        pass
    cv.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)