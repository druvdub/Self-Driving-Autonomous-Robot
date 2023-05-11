#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Int32


# setup cv bridge
bridge = CvBridge()

# get the image from the camera and convert it to a cv image, and process it


def process_image(frame):

    cv_image = bridge.imgmsg_to_cv2(frame, 'bgr8')

    # grayscale the image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # blur the image
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # # create a mask to only look at the bottom of the image
    # mask = np.zeros_like(blur)
    # height, width = blur.shape
    # mask[9*height//10:height, width//10:9*width//10] = 255
    # # triangle = np.array(
    # #     [[(0, height), (width, height), (width//2, 3*height//4)]])
    # # cv2.fillPoly(mask, triangle, 255)
    # masked_image = cv2.bitwise_and(blur, mask)

    # binary inverse the image
    _, thresh = cv2.threshold(
        blur, 127, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)

    # create a mask to only look at the bottom of the image
    mask = np.zeros_like(thresh)
    height, width = thresh.shape
    mask[9*height//10:height, width//10:8*width//10] = 255
    # triangle = np.array(
    #     [[(0, height), (width, height), (width//2, 3*height//4)]])
    # cv2.fillPoly(mask, triangle, 255)
    masked_image = cv2.bitwise_and(thresh, mask)

    # find the contours of the image
    contours, hierarchy = cv2.findContours(
        masked_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # find the position of the line with respect to the center of the image
    # if line is on the left, the position will be negative else positive
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        M = cv2.moments(c)
        if M['m00'] != 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
            line_pos = cx - width//2
            line_pub.publish(line_pos)
        else:
            line_pub.publish(0)
    # if no line is detected, shift control to other node
    else:
        # send impossible value as check
        line_pub.publish(1000)


if __name__ == '__main__':
    # initialize node
    rospy.init_node('line_detector', anonymous=True)

    # setup publishers and subscribers
    # send the line position to next node
    line_pub = rospy.Publisher('/line_position', Int32, queue_size=10)

    image_sub = rospy.Subscriber(
        '/video_source/raw', Image, queue_size=1)

    rospy.spin()


# rospy.init_node('line_follower', anonymous=True)

# # setup publishers and subscribers
# # send the line position to next node
# line_pub = rospy.Publisher('/line_position', Int32, queue_size=10)

# image_sub = rospy.Subscriber(
#     '/video_source/raw', Image, queue_size=1, callback=process_image)

# rospy.spin()
