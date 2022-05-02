#!/usr/bin/env python
from pandas import Float64Index
import rospy
from std_msgs.msg import String 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from math import pow, atan2, sqrt
import math

## Initialize global variables.
front_left = 0
front_right = 0
dt_front = 0.0001

## Initialize constants.
wheel_radius = 0.05
wheel_circumference = wheel_radius * 2 * math.pi
wheels_distance = 0.19

class PuzzleBot:
    wl=wr=0

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make 
        # sure it is a unique node (using anonymous=True).
        rospy.init_node('puzzlebot_controller', anonymous=True)

        self.car_pose = rospy.Publisher('location', Pose, queue_size=10)
        # A subscriber to the topic '/wl' and '/wr'. self.update_wx is called
        # when a message of type Pose is received.
        self.wl_subscriber = rospy.Subscriber('/wl', Float32, self.update_wl)
        self.wr_subscriber = rospy.Subscriber('/wr', Float32, self.update_wr)

        self.pose = Pose()
        self.rate = rospy.Rate(10)
        self.robot_pose()
        self.car_pose.publish(self.pose)
        rospy.spin()

    def update_wr(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wr = data

    def update_wl(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wl = data

    def robot_pose(self):
        self.pose.orientation.w = self.pose.orientation.w+wheel_radius*((self.wr-self.wl)/wheels_distance)*dt_front
        self.pose.position.x = self.pose.position.x+wheel_radius*((self.wr+self.wl)/2)*dt_front*math.cos(self.pose.orientation.w)
        self.pose.position.y = self.pose.position.y+wheel_radius*((self.wr+self.wl)/2)*dt_front*math.sin(self.pose.orientation.w)

        #self.car_pose.publish(self.pose)

if __name__ == '__main__':
    try:
        x = PuzzleBot()
    except rospy.ROSInterruptException:
        pass