#!/usr/bin/env python
from pandas import Float64Index
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster, TransformListener,Buffer
from std_msgs.msg import String 
from std_msgs.msg import Float32, Empty
from geometry_msgs.msg import Twist, Pose, Point, TransformStamped, Vector3, Quaternion
#from math import pow, atan2, sqrt
#from math import pi, cos, sin
import math
import numpy as np

## Initialize constants.
dt_front = 0.01
wheel_radius = 0.05
wheel_circumference = wheel_radius * 2 * math.pi
wheels_distance = 0.19

class Odometry:
    #Initial values for left wheel and right wheel
    wl=0.0
    wr=0.0
# Initialization of pose in free space, composed of position and orientation.
    pose = Pose()

    def update_wr(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wr = data.data

    def update_wl(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wl = data.data
    #
    def robot_pose(self):
        # Converts a given set of Euler angles, to the corresponding quaternion.
        euler = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        yaw = euler[2]

        #self.pose.orientation.w = self.pose.orientation.w+wheel_radius*((self.wr-self.wl)/wheels_distance)*dt_front
        sumVel = self.wr + self.wl
        # Calculates robot's actual pose
        self.pose.position.x = self.pose.position.x+(wheel_radius*((sumVel)/2)*dt_front*math.cos(yaw))
        self.pose.position.y = self.pose.position.y+(wheel_radius*((sumVel)/2)*dt_front*math.sin(yaw))
        if(yaw>math.pi or yaw<-math.pi):
            yaw*=-1
        
        yaw += wheel_radius*((self.wr-self.wl)/wheels_distance)*dt_front
        rospy.loginfo(yaw)
        quat = quaternion_from_euler(0, 0, yaw)
        # Gives to pose robots orientation
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]

    # The broadTf (Broadcast Transform) function for the robot pose message 
    # broadcasts this robot's translation and rotation, and publishes it as
    # a transform from frame "base_origin" to frame "base_footprint".
    def broadTf(self):
        TB = TransformBroadcaster()
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = "base_origin"
        TS.child_frame_id = "base_footprint"
        TS.transform.translation = Vector3(self.pose.position.x, self.pose.position.y, self.pose.position.z)
        TS.transform.rotation = self.pose.orientation
        TB.sendTransform(TS)
        
    #Function that return robot pose
    def getPose(self):
        return self.pose
    
    def restart(self,_):
        self.pose = Pose()


def main():
    #Create node of Odometry
    rospy.init_node('Odom', anonymous=True)
    rate = rospy.Rate(100)
""" This represents an estimate of a position and velocity in free space.  
    The pose in this message should be specified in the coordinate frame 
    given by header.frame_id. The twist in this message should be specified 
    in the coordinate frame given by the child_frame_id"""
    odom = Odometry()
    # Both /wl & /wr are topics published by the puzzlebot referring to the velocity of the wheels
    rospy.Subscriber('/wl', Float32, odom.update_wl)
    rospy.Subscriber('/wr', Float32, odom.update_wr)
    #Restart Odometry pose to initial position
    rospy.Subscriber("position/restart", Empty, odom.restart)

    while not rospy.is_shutdown():
        odom.robot_pose()
        odom.broadTf()
        rate.sleep()


if __name__ == '__main__':
    main()
