#!/usr/bin/env/ python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool


# setup cv bridge
bridge = CvBridge()


def is_color_within_range(color, color_range:  list) -> bool:
    hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)[0][0]
    return hsv_color[0] >= color_range[0][0] and hsv_color[1] >= color_range[0][1] and hsv_color[2] >= color_range[0][2] and hsv_color[0] <= color_range[1][0] and hsv_color[1] <= color_range[1][1] and hsv_color[2] <= color_range[1][2]


def process_keypoints(keypoints: list, color_range: list, cv_image: np.ndarray) -> bool:
    for keypoint in keypoints:
        x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
        w, h = int(keypoint.size), int(keypoint.size)
        roi = cv_image[y-h//2:y+h//2, x-w//2:x+w//2]
        avg_color = np.mean(roi, axis=(0, 1))
        if is_color_within_range(avg_color, color_range):
            return True
    return False


# get the image from the camera and convert it to a cv image, and process it
def process_image(frame):

    cv_image = bridge.imgmsg_to_cv2(frame, 'bgr8')

    # convert to hsv
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    # define color ranges for green and red
    lower_green = np.array([30, 51, 51])
    upper_green = np.array([99, 255, 255])
    lower_red = np.array([144, 35, 20])
    upper_red = np.array([179, 255, 255])

    # create masks for green and red
    green_mask = cv2.inRange(hsv, lower_green, upper_green)
    red_mask = cv2.inRange(hsv, lower_red, upper_red)

    # apply morphological transformations to remove noise
    kernel = np.ones((5, 5), np.uint8)
    green_mask = cv2.erode(green_mask, kernel, iterations=1)
    green_mask = cv2.dilate(green_mask, kernel, iterations=2)
    red_mask = cv2.erode(red_mask, kernel, iterations=1)
    red_mask = cv2.dilate(red_mask, kernel, iterations=2)

    # detect blobs in the binary masks
    params = cv2.SimpleBlobDetector_Params()
    params.filterByInertia = True
    params.minInertiaRatio = 0.1
    params.filterByConvexity = False
    params.filterByCircularity = True
    params.minCircularity = 0.5
    params.filterByArea = True
    params.minArea = 200
    detector = cv2.SimpleBlobDetector_create(params)

    red_keypoints = detector.detect(cv2.bitwise_not(red_mask))
    green_keypoints = detector.detect(cv2.bitwise_not(green_mask))

    # draw blobs on the original image
    blob = cv2.drawKeypoints(cv_image, red_keypoints, np.array(
        []), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    blob = cv2.drawKeypoints(blob, green_keypoints, np.array(
        []), (0, 255, 0), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

    # define color ranges for traffic lights
    red_color_range = [lower_red, upper_red]
    green_color_range = [lower_green, upper_green]

    # process blobs to detect traffic lights
    red_detected = process_keypoints(
        red_keypoints, red_color_range, cv_image) if red_keypoints else False
    green_detected = process_keypoints(
        green_keypoints, green_color_range, cv_image) if green_keypoints else False

    # publish the results
    if red_detected and not green_detected:
        traffic_light_pub.publish(0)
    elif green_detected and not red_detected:
        traffic_light_pub.publish(1)
    else:
        traffic_light_pub.publish(1)

    # show the image
    cv2.imshow('traffic_light_detector', blob)


if __name__ == '__main__':
    rospy.init_node('traffic_detection', anonymous=True)

    # send boolean status of traffic light
    traffic_light_pub = rospy.Publisher(
        '/traffic_light_status', Bool, queue_size=1)

    # subscribe to the camera image
    image_sub = rospy.Subscriber(
        '/video_source/raw', Image, queue_size=1, callback=process_image)

    rospy.spin()
