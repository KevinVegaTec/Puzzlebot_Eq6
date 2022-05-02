#!/usr/bin/env python
import rospy
from pandas import Float64Index
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster, TransformListener,Buffer
from std_msgs.msg import String 
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist, Pose, Point, TransformStamped, Vector3, Quaternion
import math
import numpy as np

class Control:
    pose = Pose()
    desPose = Point()
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    vel = Twist()
    #kpl = 0.35
    #kpt = 0.05
    kpl = 0.75
    kpt = 0.40

    def __init__(self):
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer)
    
    def setDesiredPosition(self, data):
        self.desPose = data
        self.broadcastTransform()
        while(not self.startControl()):
            None
        self.setVelocities(Twist())
    
    def setPosition(self, data):
        self.pose = data

    def startControl(self):
        transformObject = self.tf_buffer.lookup_transform("base_footprint", "point", rospy.Time())
        rospy.loginfo(transformObject)

        TF = transformObject.transform.translation
        self.PID(TF)
        return(abs(TF.x)<0.01 and abs(TF.y)<0.01)

    def PID(self, error):
        self.vel.linear.x = self.kpl*error.x

        yaw = math.atan2(error.y,error.x)
        self.vel.angular.z = self.kpt*yaw
        self.setVelocities(self.vel)

    def setVelocities(self, speed):
        self.posePub.publish(speed)
    
    def broadcastTransform(self):
        BR = TransformBroadcaster()
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = "base_origin"
        TS.child_frame_id = "point"
        TS.transform.translation = Vector3(self.desPose.x, self.desPose.y, self.desPose.z)
        TS.transform.rotation = Quaternion(0,0,0,1)
        BR.sendTransform(TS)


def main():
    rospy.init_node('Control', anonymous=True)
    rate = rospy.Rate(100)

    control = Control()
    rospy.Subscriber("DesiredPosition", Point, control.setDesiredPosition)
    rospy.Subscriber("Position",Pose, control.setPosition)
    while not rospy.is_shutdown():
        control.broadcastTransform()
        rate.sleep()

if __name__ == "__main__":
    main()