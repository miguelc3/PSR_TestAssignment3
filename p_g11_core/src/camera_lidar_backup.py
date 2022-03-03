#!/usr/bin/env python3

# =============================
# Import pkgs
# =============================
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
import tf2_ros
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from time import time
import subprocess


class CamImg():

    def __init__(self):

        # Get node name (to change the node name in the terminal: __name:=...)
        self.name = rospy.get_name()
        self.name = self.name.strip('/')
        # print('My name is ' + self.name)

        self.bridge = CvBridge()
        self.goal_active = False
        self.hunting = False
        self.running = False
        self.angle = 0
        self.speed = 0
        self.cy = 0

        # lidar
        # self.finding_goal = True
        self.collision_active = False
        self.colision_subscriber = rospy.Subscriber('/' + self.name + '/scan', LaserScan, self.callbackLaserReceived)

        # Segment color limits
        self.blue_limits = {'B': {'max': 255, 'min': 100}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 50, 'min': 0}}
        self.red_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 50, 'min': 0}, 'R': {'max': 255, 'min': 100}}
        self.green_limits = {'B': {'max': 50, 'min': 0}, 'G': {'max': 255, 'min': 100}, 'R': {'max': 50, 'min': 0}}

        # =============================
        # Decide team
        # =============================
        red_team = rospy.get_param('/red_players')
        green_team = rospy.get_param('/green_players')
        blue_team = rospy.get_param('/blue_players')
        self.my_team = None  # Just to initialize the variable

        if self.name in red_team:
            self.my_team = 'red'
            self.my_teams_player = red_team
            self.prey_team_players = green_team
            self.hunter_team_players = blue_team
        elif self.name in green_team:
            self.my_team = 'green'
            self.my_teams_player = green_team
            self.prey_team_players = blue_team
            self.hunter_team_players = red_team
        elif self.name in blue_team:
            self.my_team = 'blue'
            self.my_teams_player = blue_team
            self.prey_team_players = red_team
            self.hunter_team_players = green_team
        else:
            rospy.logerr('Something is wrong, I\'m not on the player\'s list')
            exit(0)

        # Create an instance for each player
        count = 1
        for player in red_team + green_team + blue_team:
            # Add gazebo marker text with the name of the player
            cmd = "gz marker -m 'action: ADD_MODIFY, type: TEXT, id: " + str(count) + \
                  ", scale: {x:0.3, y:0.3, z:0.3}, text: \"" + player + "\", parent: \"" + player + \
                  "::base_footprint\", pose: {position: {x:0, y:0, z:0.5}, orientation: {x:0, y:0, z:0, w:1}}'"
            self.bash(cmd, verbose=True)
            count += 1

        # print('My name is ' + self.name + '. I am team ' + self.my_team + ', I am hunting  ' +
        #       str(self.prey_team_players) + ' and fleeing from ' + str(self.hunter_team_players))

    # =============================
    # Functions
    # =============================
    def bash(self, cmd, blocking=True, verbose=False):
        # Function to diplay players names
        if verbose:
            print("Executing command: " + cmd)
        p = subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        if blocking:
            for line in p.stdout.readlines():
                print(line)
                p.wait()

    def show_image(self, img, window_name):
        # Function to show image - used to test
        cv2.namedWindow(window_name, 1)
        cv2.imshow(window_name, img)
        cv2.waitKey(3)

    def image_callback(self, img_msg):
        # Convert image to opencv
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(img_msg, "passthrough")
            self.cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
            self.segment_players(self.cv_image)
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        # self.show_image(self.cv_image, 'live image')

    def segment_players(self, cv_image):
        # Function to segment player according to the colors

        # Define maks
        self.blue_mask = cv2.inRange(cv_image,
                                     (self.blue_limits['B']['min'], self.blue_limits['G']['min'],
                                      self.blue_limits['R']['min']),
                                     (self.blue_limits['B']['max'], self.blue_limits['G']['max'],
                                      self.blue_limits['R']['max']))

        self.red_mask = cv2.inRange(cv_image,
                                    (self.red_limits['B']['min'], self.red_limits['G']['min'],
                                     self.red_limits['R']['min']),
                                    (self.red_limits['B']['max'], self.red_limits['G']['max'],
                                     self.red_limits['R']['max']))

        self.green_mask = cv2.inRange(cv_image,
                                      (self.green_limits['B']['min'], self.green_limits['G']['min'],
                                       self.green_limits['R']['min']),
                                      (self.green_limits['B']['max'], self.green_limits['G']['max'],
                                       self.green_limits['R']['max']))

        # Hunt mode
        if self.my_team == 'red':
            self.mask_hunt = self.green_mask
        elif self.my_team == 'green':
            self.mask_hunt = self.blue_mask
        elif self.my_team == 'blue':
            self.mask_hunt = self.red_mask

        self.largest_object(self.mask_hunt, 'hunting')

        # Run away mode
        if self.my_team == 'red':
            self.mask_run = self.blue_mask
        elif self.my_team == 'green':
            self.mask_run = self.red_mask
        elif self.my_team == 'blue':
            self.mask_run = self.green_mask

        self.largest_object(self.mask_run, 'running')

    def largest_object(self, mask, state):
        # Function to find objects according to the masks previously defined

        # Verify if it is running away or hunting
        if state == 'hunting':
            self.hunting = True
        elif state == 'running':
            self.running = True

        # Initialize the window of the largest object -> all black
        self.mask_largest = np.zeros(mask.shape, dtype=np.uint8)

        # Find the largest object
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        cnt = sorted(contours, key=cv2.contourArea)
        if len(cnt) > 0:
            largest = cnt[-1]
            cv2.fillPoly(self.mask_largest, pts=[largest], color=(255, 255, 255))

            # Find centroid
            m = cv2.moments(largest)
            if m['m00'] == 0:
                self.cx = 0
                cy = 0
            else:
                self.cx = int(m['m10'] / m['m00'])
                cy = int(m['m01'] / m['m00'])
        else:
            self.cx = 0
            cy = 0

        if not self.collision_active:
            if np.count_nonzero(self.mask_largest) > 0:
                self.goal_active = True
            else:
                self.goal_active = False

        self.mask_largest = cv2.circle(self.mask_largest, (self.cx, cy), 2, (0, 0, 0), -1)
        # self.show_image(self.mask_largest, 'Mask hunt largest')

    def send_command_callback(self, event):
        # print('Sending twist command')

        if not self.collision_active:
            # Hunting
            if self.hunting:
                if not self.goal_active:
                    # angular vel = 0.5 -> look for preys
                    self.speed = 0.2
                    self.angle = 0.5
                else:
                    self.drive_straight()

            elif self.running:
                # Running
                if not self.goal_active:
                    # angular vel = 0.5 -> look for preys
                    self.speed = 0.2
                    self.angle = 0.5
                else:
                    self.drive_straight_run()

        elif self.collision_active:
            self.callbackLaserReceived()

        # Build the Twist message
        twist = Twist()
        twist.linear.x = self.speed
        twist.angular.z = self.angle

        # Publish twist message
        self.publisher_command.publish(twist)

    def drive_straight(self):

        width = self.mask_largest.shape[1]
        center = round(width / 2)

        if self.cx < center:
            objective = 'need to turn left'
        else:
            objective = 'need to turn right'

        # Define angle
        if self.cx < center / 2:
            self.angle = 1
        elif self.cx < center:
            self.angle = 0.5
        elif self.cx == center:
            self.angle = 0
        elif self.cx > 0.75 * width:
            self.angle = -1
        else:
            self.angle = -0.5

        # Define speed - constant
        self.speed = 1

        print('I am ' + self.name + '. Centroid is at = ' + str(self.cx) + ', width = ' + str(width) + ' and center = '
              + str(center) + ' so I ' + objective + ' with angular velocity = ' + str(self.angle))

    def drive_straight_run(self):

        width = self.mask_largest.shape[1]
        center = round(width / 2)

        if self.cx < center:
            objective = 'need to turn right'
        else:
            objective = 'need to turn left'

        # Define angle
        if self.cx < center / 2:
            self.angle = -1
        elif self.cx < center:
            self.angle = -0.5
        elif self.cx == center:
            self.angle = 1
        elif self.cx > 0.75 * width:
            self.angle = 1
        else:
            self.angle = 0.5

        # Define speed - constant
        self.speed = 1

        print('I am ' + self.name + '. Centroid is at = ' + str(self.cx) + ', width = ' + str(width) + ' and center = '
              + str(center) + ' so I ' + objective + ' with angular velocity = ' + str(self.angle))

    def callbackLaserReceived(self, laser):
        # Function to avoid going into the walls
        h = False
        r = False

        rospy.loginfo('Received laser scan message')

        # Constants to define the distance from a wall
        thresh = 0.5
        thresh2 = 3

        angle = laser.angle_min

        # print(message.ranges[0])

        for i, range in enumerate(laser.ranges):
            angle += laser.angle_increment * i
            if range < thresh:
                self.speed = 0.2
                self.angle = -angle

        # Checking the message received by the laserscan
        # for i, range in enumerate(laser.ranges):
        #     theta = laser.angle_min + laser.angle_increment * i
        #     #  Checking all the angles to see if it's close to a wall
        #     if range < thresh:
        #         t = time()
        #         self.collision_active = True
        #         if self.hunting:
        #             h = True
        #             self.hunting = False
        #         elif self.running:
        #             r = True
        #             self.running = False

        #         if laser.ranges[0] < thresh:
        #             self.speed = -0.3
        #             self.angle = 0
        #         elif laser.ranges[180] < thresh:
        #             self.speed = 0.3
        #             self.angle = 0
        #         elif laser.ranges[30] < thresh:
        #             self.speed = 0.3
        #             self.angle = -0.5
        #         elif laser.ranges[330] < thresh:
        #             self.speed = 0.3
        #             self.angle = 0.5
        #     # else:
        #     #     t = 0
        #     elif range > thresh2:
        #         self.collision_active = False

        #         if h:
        #             self.hunting = True
        #             h = False
        #         elif r:
        #             self.running = True
        #             r = False


# =============================
# Main function
# =============================
def main():
    name = rospy.init_node('p_g11', anonymous=False)
    camImg = CamImg()
    while not rospy.is_shutdown():
        rospy.spin()


if __name__ == '__main__':
    main()