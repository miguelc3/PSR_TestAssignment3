#!/usr/bin/env python3

# Set a goal to our robot - in rviz

import rospy
from std_msgs.msg import String
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
import tf2_geometry_msgs
import copy
import math


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


    def goalReceivedCallback(self, msg):

        print('Received new goal on frame_id ' + msg.header.frame_id)

        target_frame = self.name + '/odom'

        # Receiving a goal to move the robot
        try:
            self.goal = self.tf_buffer.transform(msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.goal_active = False
            rospy.logerr('Could not transform goal from ' + msg.header.frame_id + ' to ' + target_frame + '. Will ignore this goal')

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

        # Defining the angle to go to reach the goal
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


def main():

    # =================================
    # INITIALIZATION
    # =================================
    rospy.init_node('p_g11', anonymous=False)

    driver = Driver()
    rospy.spin()


if __name__ == '__main__':
    main()
