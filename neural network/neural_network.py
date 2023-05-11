#!/usr/bin/env python

import rospy
import numpy as np
import tensorflow as tf
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int32
import cv2


# setup cv bridge
bridge = CvBridge()

# load the model
model = tf.keras.models.load_model('VGG.h5')


# get the image from the camera and convert it to a cv image, and process it
def process_image(frame):

    image = bridge.imgmsg_to_cv2(frame, 'bgr8')

    # check if the image is empty
    if image is None:
        traffic_sign_pub.publish(-1)

    # resize the image
    image = cv2.resize(image, (32, 32))

    # pre-process the image
    # normalize the image
    image = image/255.0

    # convert the image to a 4D tensor
    image = np.expand_dims(image, axis=0)

    # predict image class
    prediction = model.predict(image)

    # get the class with the highest probability
    prediction = np.argmax(prediction)

    # publish the class
    traffic_sign_pub.publish(prediction)


if __name__ == '__main__':
    # initialize the node
    rospy.init_node('traffic_sign_recognition')

    # create a publisher to publish the traffic sign
    traffic_sign_pub = rospy.Publisher(
        '/traffic_sign_status', Int32, queue_size=1)

    # create a subscriber to the camera
    rospy.Subscriber('/crop_filter', Image, process_image)

    # spin
    rospy.spin()
