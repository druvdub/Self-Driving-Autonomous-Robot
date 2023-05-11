#!usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# setup cv bridge
bridge = CvBridge()

# get the image from the camera and convert it to a cv image, and process it
# crop image to get traffic sign only


def crop_image(frame):

    cv_image = bridge.imgmsg_to_cv2(frame, 'bgr8')

    # grayscale image
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    # apply gaussian blur
    blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # apply thresholding
    ret, t = cv2.threshold(
        blur, 0, 255, cv2.THRESH_BINARY_INV+cv2.THRESH_OTSU)

    v = np.median(t)
    sigma = 0.2
    lower = int(max(0, (1.0 - sigma) * v))
    upper = int(min(255, (1.0 + sigma) * v))

    # apply canny edge detection
    canny = cv2.Canny(t, lower, upper, apertureSize=3, L2gradient=True)

    # mask bottom half of image
    mask = np.zeros(canny.shape, dtype=np.uint8)
    mask[:canny.shape[0]//2, :] = 255

    # apply mask
    masked = cv2.bitwise_and(canny, canny, mask=mask)

    # blob detection
    params = cv2.SimpleBlobDetector_Params()
    params.filterByArea = True
    params.minArea = 5000
    params.maxArea = 1000000
    params.filterByCircularity = True
    params.minCircularity = 0.6
    params.filterByConvexity = True
    params.minConvexity = 0.87
    params.filterByInertia = True
    params.minInertiaRatio = 0.01
    detector = cv2.SimpleBlobDetector_create(params)

    # detect blobs
    keypoints = detector.detect(canny)

    if len(keypoints) == 0:
        crop_pub.publish(Image())
        return

    # get the largest blob
    largest_blob = max(keypoints, key=lambda x: x.size)

    # draw blobs on image
    kp = cv2.drawKeypoints(cv_image, [largest_blob], np.array(
        []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # draw bound box around blob
    x, y = largest_blob.pt
    r = largest_blob.size // 2
    x1 = int(x - r)
    x2 = int(x + r)
    y1 = int(y - r)
    y2 = int(y + r)

    # crop image
    crop = cv_image[y1:y2, x1:x2]

    # publish cropped image
    crop_pub.publish(bridge.cv2_to_imgmsg(crop, 'bgr8'))


if __name__ == '__main__':
    rospy.init_node('crop_filter', anonymous=True)

    crop_pub = rospy.Publisher('/crop_filter', Image, queue_size=1)

    rospy.Subscriber('/video_source/raw', Image, crop_image)
    rospy.spin()
