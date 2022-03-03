#!/usr/bin/env python3

# Set a goal to our robot - in rviz

import rospy
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2
from time import time

from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs
import copy
import math


# publisher = rospy.Publisher('/p_g11/point_cloud', PointCloud2)

class Driver():

    def __init__(self):

        # Define a goal Pose to witch the robot should move
        self.goal = PoseStamped()
        self.goal_active = False

        self.angle = 0
        self.speed = 0

        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # remove intial '/'
        rospy.loginfo('My player name is ' + self.name)

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

        self.finding_goal = True
        self.colision_active = False
        self.colision_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.callbackMessageReceived)

    def goalReceivedCallback(self, msg):

        print('Received new goal on frame_id ' + msg.header.frame_id)

        target_frame = self.name + '/odom'

        if self.finding_goal:
            try:
                self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
                self.goal_active = True
                self.finding_goal = False
                rospy.logwarn('Setting new goal')
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                self.goal_active = False
                self.finding_goal = True
                rospy.logerr(
                    'Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this goal')

        # print('Received new goal')
        # self.goal = copy.copy(msg)  # Store goal
        # self.goal_active = True

    def driveSraight(self, min_speed=0.2, max_speed=0.5):

        goal_copy = copy.deepcopy(self.goal)  # make sure we don't change the stamp field of the goal
        goal_copy.header.stamp = rospy.Time.now()

        # goal_tf = tf2_geometry_msgs.PoseStamped()
        # goal_tf.header.stamp = rospy.Time.now()
        # goal_tf.header.frame_id = self.goal.header.frame_id

        print('Transforming pose')
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))
        print('Pose transformed')

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)
        distance_to_goal = math.sqrt(x ** 2 + y ** 2)

        self.speed = max(min_speed, 0.5 * distance_to_goal)  # Limit min speed
        self.speed = min(max_speed, self.speed)  # Limit max speed

    def sendCommandCallback(self, event):

        print('Sending twist command')
        # print(self.goal_active)
        # print(self.finding_goal)
        # print(self.colision_active)

        if not self.colision_active:
            # Decision outputs a speed (linear velocity) and an angle
            if not self.goal_active:  # no goal, no movement
                self.angle = 0
                self.speed = 0
            else:
                self.driveSraight()

        # Build the Twist message
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        # Publish twist message
        self.publisher_command.publish(twist)

    def callbackMessageReceived(self, msg):
        rospy.loginfo('Received laser scan message')

        time_avoid = 20
        thresh = 0.8
        thresh2 = 5

        for i, range in enumerate(msg.ranges):
            theta = msg.angle_min + msg.angle_increment * i
            if range < thresh:
                t = time()
                self.goal_active = False
                self.finding_goal = False
                self.colision_active = True

                if msg.ranges[0] < thresh:
                    self.speed = -0.3
                    self.angle = 0
                elif msg.ranges[180] < thresh:
                    self.speed = 0.3
                    self.angle = 0
                elif msg.ranges[30] < thresh:
                    self.speed = 0.3
                    self.angle = -0.5
                elif msg.ranges[330] < thresh:
                    self.speed = 0.3
                    self.angle = 0.5
            # else:
            #     t = 0
            # elif range > thresh2:
            #     self.colision_active = False
            #     self.finding_goal = True


            # if t > time_avoid:
            #     self.colision_active = False
            #     self.speed = 0
            #     self.angle = 0


def main():
    # =================================
    # INITIALIZATION
    # =================================
    rospy.init_node('p_g11', anonymous=False)

    # rospy.Subscriber('/p_g11/scan', LaserScan, callbackMessageReceived)

    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
