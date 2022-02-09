#!/usr/bin/env python3

import copy
import math
import rospy
import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped


class Driver:

    def __init__(self):

        self.name = rospy.get_name()
        self.name = self.name.strip('/')  # remove initial /
        print('My player name is ' + self.name)

        self.publisher_command = rospy.Publisher('/' + self.name + '/cmd_vel', Twist, queue_size=1)

        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.timer = rospy.Timer(rospy.Duration(0.1), self.sendCommandCallback)

        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goalReceivedCallback)

        self.goal = PoseStamped()
        self.goal_active = False

        self.angle = 0
        self.speed = 0

        self.minimum_speed = 0.5
        self.maximum_speed = 2

    def goalReceivedCallback(self, goal_msg):

        # TODO verify is goal is on odom frame
        print('Received new goal on frame id' + goal_msg.header.frame_id)
        target_frame = self.name + '/odom'
        try:

            self.goal = self.tf_buffer.transform(goal_msg, target_frame, rospy.Duration(1))
            self.goal_active = True
            rospy.logwarn('Setting new goal')
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            goal_active = False
            rospy.logerr(
                'Could not transform goal from ' + goal_msg.header.frame_id + ' to ' + target_frame + '. Will ignore this '
                                                                                                 'goal.')

    def driveStraight(self, goal, minimum_speed=0.1, maximum_speed=0.5):
        goal_copy = copy.deepcopy(self.goal)  # make sure we don't change the stamp field of the goal
        goal_copy.header.stamp = rospy.Time.now()

        # goal_tf = tf2_geometry_msgs.PoseStamped()
        # goal_tf.header.stamp = rospy.Time.now()
        # goal_tf.header.frame_id = self.goal.header.frame_id

        print('Transforming pose')
        goal_in_base_link = self.tf_buffer.transform(goal_copy, self.name + '/base_footprint', rospy.Duration(1))
        print('Pose trasnformed')

        x = goal_in_base_link.pose.position.x
        y = goal_in_base_link.pose.position.y

        self.angle = math.atan2(y, x)

        distance_to_goal = math.sqrt(x ** 2 + y ** 2)
        self.speed = max(self.minimum_speed, 0.5 * distance_to_goal)  # limit minimum speed
        self.speed = min(self.maximum_speed, self.speed)  # limit maximum speed

    def sendCommandCallback(self, event):
        print('Sending twist command')

        # distance_to_goal = self.computeDistancetoGoal(self.goal)
        # if distance_to_goal < 0.3:
        #     rospy.logwarn('The robot has achieved the goal!!!')
        #     self.goal_active = False

        if not self.goal_active:  # no goal, no movement
            self.angle = 0
            self.speed = 0
        else:
            self.driveStraight()

        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle
        self.publisher_command.publish(twist)

    def computeDistancetoGoal(self, goal):
        pass


def main():
    rospy.init_node('p_g11_driver', anonymous=False)

    driver = Driver()

    rate = rospy.Rate(10)

    rospy.spin()


if __name__ == '__main__':
    main()
