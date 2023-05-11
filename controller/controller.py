#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Bool, Int32
from geometry_msgs.msg import Twist


class Controller:
    FOLLOW_LINE = "pid_control"
    FOLLOW_TRAFFIC_SIGN = "fixed_control"

    def __init__(self):

        self.FOLLOW_LINE = "pid_control"
        self.FOLLOW_TRAFFIC_SIGN = "fixed_control"

        rospy.init_node('controller')

        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.line_pos_sub = rospy.Subscriber(
            '/line_position', Int32, self.line_pos_callback, queue_size=10)
        self.traffic_detected_sub = rospy.Subscriber(
            '/traffic_light_status', Bool, self.traffic_detected_callback, queue_size=10)
        self.traffic_sign_sub = rospy.Subscriber(
            '/traffic_sign_status', Int32, self.traffic_sign_callback, queue_size=10)

        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0

        # default values
        self.traffic_light_status = True
        self.traffic_sign_status = -1
        self.line_pos = 0

        self.state = self.FOLLOW_LINE
        self.is_fixed_control_running = False

        # Initialize PID parameters
        self.Kp = 0.038
        self.Ki = 0.0
        self.Kd = 0.0
        self.prev_error = 0.0
        self.integral = 0.0

    def line_pos_callback(self, msg):
        self.line_pos = msg.data
        if self.line_pos == 1000:
            self.state = self.FOLLOW_TRAFFIC_SIGN

    def traffic_detected_callback(self, msg):
        self.traffic_light_status = msg.data

    def traffic_sign_callback(self, msg):
        # if the traffic sign is detected, then set the traffic sign status
        # if a new traffic sign is detected, update the traffic sign status
        # else don't update till fixed control is done
        if msg.data != -1 and not self.is_fixed_control_running:
            self.traffic_sign_status = msg.data
        elif self.is_fixed_control_running:
            self.traffic_sign_status = msg.data
            self.is_fixed_control_running = False

    def state_machine(self):
        if self.traffic_light_status:
            if self.state == self.FOLLOW_LINE:
                self.pid_control()
            elif self.state == self.FOLLOW_TRAFFIC_SIGN:
                self.fixed_control()
                self.is_fixed_control_running = True
                self.state = self.FOLLOW_LINE
        else:
            self.stop()

    def stop(self):
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.angular.z = 0.0
        self.pub_cmd_vel.publish(self.cmd_vel)

    def fixed_control(self):
        # STOP
        if self.traffic_sign_status == 0:
            self.stop()
        # ROAD WORKS AHEAD
        elif self.traffic_sign_status == 1:
            # slow down
            self.cmd_vel.linear.x = 0.2
            self.pub_cmd_vel.publish(self.cmd_vel)
        # TURN RIGHT AHEAD
        elif self.traffic_sign_status == 2:
            # move forward, stop after 1 sec, turn right, move forward
            self.cmd_vel.linear.x = 0.5
            self.pub_cmd_vel.publish(self.cmd_vel)
            rospy.sleep(rospy.Duration(1))
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = -0.5
            self.pub_cmd_vel.publish(self.cmd_vel)
            rospy.sleep(rospy.Duration(1))
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.angular.z = 0.0
            self.pub_cmd_vel.publish(self.cmd_vel)
        # TURN LEFT AHEAD
        elif self.traffic_sign_status == 3:
            # move forward, stop after 1 sec, turn left, move forward
            self.cmd_vel.linear.x = 0.5
            self.pub_cmd_vel.publish(self.cmd_vel)
            rospy.sleep(rospy.Duration(1))
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.angular.z = 0.5
            self.pub_cmd_vel.publish(self.cmd_vel)
            rospy.sleep(rospy.Duration(1))
            self.cmd_vel.linear.x = 0.5
            self.cmd_vel.angular.z = 0.0
            self.pub_cmd_vel.publish(self.cmd_vel)
        # GO STRAIGHT AHEAD
        elif self.traffic_sign_status == 4:
            # move forward
            self.cmd_vel.linear.x = 0.5
            self.pub_cmd_vel.publish(self.cmd_vel)
        # ROUNDABOUT AHEAD
        elif self.traffic_sign_status == 5:
            pass

    def pid_control(self):
        error = self.line_pos
        proportional = self.Kp * error
        self.integral += error * self.Ki
        derivative = (error - self.prev_error) / self.Kd
        result = proportional + self.integral + derivative
        self.prev_error = error

        self.cmd_vel.linear.x = 0.5
        self.cmd_vel.angular.z = result * 0.01

        self.pub_cmd_vel.publish(self.cmd_vel)


if __name__ == '__main__':
    try:
        controller = Controller()
        while not rospy.is_shutdown():
            controller.state_machine()
    except rospy.ROSInterruptException:
        pass
