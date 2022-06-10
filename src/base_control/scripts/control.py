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
    # Initialization of pose in free space, composed of position and orientation.
    pose = Pose()
    # This contains the position of a point in free space
    desPose = Point()
    # Publishes robot pose
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    # This expresses velocity in free space broken into its linear and angular parts.
    vel = Twist()
    # Initialize PID parameters
    #kpl = 0.35
    #kpt = 0.05
    kpl = 0.75
    kpt = 0.40

    def __init__(self):
        # Stores known frames and offers a ROS service, "tf_frames", 
        # which responds to client requests with a response containing a tf2_msgs
        self.tf_buffer = Buffer()
        # Transform Listener to get access to frame transformations.
        self.listener = TransformListener(self.tf_buffer)
    
    #Function to set the desired position for the robot
    def setDesiredPosition(self, data):
        self.desPose = data
        self.broadcastTransform()
        while(not self.startControl()):
            None
        self.setVelocities(Twist())
    
    def setPosition(self, data):
        self.pose = data

    def startControl(self):
        # Looks for transform
        transformObject = self.tf_buffer.lookup_transform("base_footprint", "point", rospy.Time())
        rospy.loginfo(transformObject)
        # Transforms the translation to set it into a PID controller
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
    
    # Transform robot's translation and rotation
    def broadcastTransform(self):
        BR = TransformBroadcaster()
        # This expresses a transform from coordinate frame header.frame_id
        # to the coordinate frame child_frame_id
        TS = TransformStamped()
        TS.header.stamp = rospy.Time.now()
        TS.header.frame_id = "base_origin"
        TS.child_frame_id = "point"
        # Insert the 3 point position in vector form to the tf translation
        TS.transform.translation = Vector3(self.desPose.x, self.desPose.y, self.desPose.z)
        TS.transform.rotation = Quaternion(0,0,0,1) #x,y,z,w where w is for angle
        BR.sendTransform(TS)


def main():
    rospy.init_node('Control', anonymous=True)
    rate = rospy.Rate(100)
    # Initiate the class
    control = Control()
    #Subscribe ans listen for both topics
    rospy.Subscriber("DesiredPosition", Point, control.setDesiredPosition)
    rospy.Subscriber("Position",Pose, control.setPosition)
    while not rospy.is_shutdown():
        control.broadcastTransform()
        rate.sleep()

if __name__ == "__main__":
    main()
