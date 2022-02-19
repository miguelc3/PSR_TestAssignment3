#!/usr/bin/env python3
import math

import rospy
import std_msgs.msg
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from sensor_msgs import point_cloud2

import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import copy
import math


# publisher = rospy.Publisher('/p_g11/point_cloud', PointCloud2)


# def callbackMessageReceived(self, msg):
#     rospy.loginfo('Received laser scan message')
#
#     header = std_msgs.msg.Header(seq=msg.header.seq, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
#     fields = [PointField('x', 0, PointField.FLOAT32, 1),
#               PointField('y', 4, PointField.FLOAT32, 1),
#               PointField('z', 8, PointField.FLOAT32, 1)]
#
#     # convert from polar coordinates to cartesian and fill the point cloud
#     points = []
#     z = 0
#     for idx, range in enumerate(msg.ranges):
#         theta = msg.angle_min + msg.angle_increment * idx
#         x = range * math.cos(theta)
#         y = range * math.sin(theta)
#         points.append([x, y, z])
#
#     pc2 = point_cloud2.create_cloud(header, fields, points)  # create point_cloud2 data structure
#     publisher.publish(pc2)  # publish (will automatically convert from point_cloud2 to Pointcloud2 message)
#     rospy.loginfo('Published PointCloud2 msg')
#
#     for point in range(points):
#         dist = math.sqrt(point[1] ** 2 + point[2] ** 2)
#         if dist > 0.5:
#             self.w


# def main():
#     rospy.init_node('lidar_subscriber', anonymous=False)
#
#     rospy.Subscriber('/p_g11/scan', LaserScan, callbackMessageReceived)
#
#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()
#
#
# if __name__ == '__main__':
#     main()

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

        self.colision_active = False
        # self.colision_subscriber = rospy.Subscriber('/p_g11/scan', LaserScan, self.callbackMessageReceived)

    def goalReceivedCallback(self, msg):

        print('Received new goal on frame_id ' + msg.header.frame_id)

        target_frame = self.name + '/odom'

        try:
            # if not self.colision_active:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr(
                'Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this goal')

        # print('Received new goal')
        # self.goal = copy.copy(msg)  # Store goal
        # self.goal_active = True

    def driveSraight(self, min_speed=0.2, max_speed=1):

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

    # def callbackMessageReceived(self, msg):
    #     rospy.loginfo('Received laser scan message')
    #
    #     header = std_msgs.msg.Header(seq=msg.header.seq, stamp=msg.header.stamp, frame_id=msg.header.frame_id)
    #     fields = [PointField('x', 0, PointField.FLOAT32, 1),
    #               PointField('y', 4, PointField.FLOAT32, 1),
    #               PointField('z', 8, PointField.FLOAT32, 1)]
    #
    #     # convert from polar coordinates to cartesian and fill the point cloud
    #     points = []
    #     z = 0
    #     for idx, range in enumerate(msg.ranges):
    #         theta = msg.angle_min + msg.angle_increment * idx
    #         x = range * math.cos(theta)
    #         y = range * math.sin(theta)
    #         points.append([x, y, z])
    #
    #     pc2 = point_cloud2.create_cloud(header, fields, points)  # create point_cloud2 data structure
    #     publisher.publish(pc2)  # publish (will automatically convert from point_cloud2 to Pointcloud2 message)
    #     rospy.loginfo('Published PointCloud2 msg')
    #
    #     # for point in range(points):
    #     #     dist = math.sqrt(point[1] ** 2 + point[2] ** 2)
    #     #     if dist > 0.5:
    #     #         self.goal_active = False
    #     #         self.colision_active = True
    #     #         self.speed = self.speed/2
    #     #         self.angle = -self.angle


def main():
    # =================================
    # INITIALIZATION
    # =================================
    rospy.init_node('p_g11', anonymous=False)

    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
