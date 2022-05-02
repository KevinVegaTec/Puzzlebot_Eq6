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
    wl=0.0
    wr=0.0

    pose = Pose()
    """We want to implement control and odom in the same script by the implementation 
    of action servers and msgs, that why we have our publisher here, as future indicator"""
    #rospy.Publisher('cmd_vel', Twist, queue_size=10)
    #vel = Twist()

    def update_wr(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wr = data.data

    def update_wl(self, data):
        """Callback function which is called when a new message of type Float32 is
        received by the subscriber."""
        self.wl = data.data
    
    def robot_pose(self):
        euler = euler_from_quaternion([self.pose.orientation.x, self.pose.orientation.y, self.pose.orientation.z, self.pose.orientation.w])
        yaw = euler[2]

        #self.pose.orientation.w = self.pose.orientation.w+wheel_radius*((self.wr-self.wl)/wheels_distance)*dt_front
        sumVel = self.wr + self.wl
        self.pose.position.x = self.pose.position.x+(wheel_radius*((sumVel)/2)*dt_front*math.cos(yaw))
        self.pose.position.y = self.pose.position.y+(wheel_radius*((sumVel)/2)*dt_front*math.sin(yaw))
        if(yaw>math.pi or yaw<-math.pi):
            yaw*=-1
        
        yaw += wheel_radius*((self.wr-self.wl)/wheels_distance)*dt_front
        rospy.loginfo(yaw)
        quat = quaternion_from_euler(0, 0, yaw)
        self.pose.orientation.x = quat[0]
        self.pose.orientation.y = quat[1]
        self.pose.orientation.z = quat[2]
        self.pose.orientation.w = quat[3]

    def broadTf(self):
        TB = TransformBroadcaster()
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = "base_origin"
        TS.child_frame_id = "base_footprint"
        TS.transform.translation = Vector3(self.pose.position.x, self.pose.position.y, self.pose.position.z)
        TS.transform.rotation = self.pose.orientation
        TB.sendTransform(TS)

    def getPose(self):
        return self.pose
    
    def restart(self,_):
        self.pose = Pose()


def main():
    rospy.init_node('Odom', anonymous=True)
    rate = rospy.Rate(100)
    odom = Odometry()
    rospy.Subscriber('/wl', Float32, odom.update_wl)
    rospy.Subscriber('/wr', Float32, odom.update_wr)
    rospy.Subscriber("position/restart", Empty, odom.restart)

    while not rospy.is_shutdown():
        odom.robot_pose()
        odom.broadTf()
        rate.sleep()


if __name__ == '__main__':
    main()