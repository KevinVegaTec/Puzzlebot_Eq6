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



def callback(data):
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    speed = Twist()

    y_th = 0.2
    x_th = 0.2

    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    alligned = False
    if (data.x < -x_th): 
        speed.angular.z = -data.x/20
        rospy.logwarn("Turning right")

    elif (data.x > x_th):
        speed.angular.z = -data.x/20
        rospy.logwarn("Turning left")

    else: 
        speed.angular.z = 0
        alligned = True

    if alligned == True:
        speed.linear.x = 1/data.z
        
    else:
        speed.linear.x = 0
    
    if data.z > 50:
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0


    posePub.publish(speed)

def main():
    rospy.init_node('Point_follower', anonymous=True)
    rate = rospy.Rate(100)
    
    # posePub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("/blob/point_blob", Point, callback)

    rospy.spin()

if __name__ == "__main__":
    main()