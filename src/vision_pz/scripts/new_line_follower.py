#!/usr/bin/env python

from ftplib import MAXLINE
import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, Point

class Follower:
  def __init__(self):
    self.bridge = CvBridge()
    #cv2.namedWindow("window", 1)
    self.image_sub = rospy.Subscriber('/video_source/raw', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                       Twist, queue_size=1)
    self.image_pub = rospy.Publisher("processed_image",
                                        Image,queue_size=1)
    self.mask_pub = rospy.Publisher("mask_image", Image, queue_size=1)
    self.line_center = rospy.Publisher("line_center", Float32,queue_size=1)
    rospy.loginfo("follower initialized")


    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    image = cv2.rotate(image, cv2.ROTATE_180)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    image = cv2.GaussianBlur(image, (3,3), 3)

    edged = cv2.Canny(image,150,250)
    height = edged.shape[0]
    width = edged.shape[1]
    # rospy.logwarn(height)
    # rospy.logwarn("fl")
    # rospy.logwarn(width)
    # Defining Triangular ROI: The values will change as per your camera mounts
    triangle = np.array([[(0, height), (width, height), (width-160, int(height/5))]])
    # creating black image same as that of input image
    black_image = np.zeros_like(edged)
    # Put the Triangular shape on top of our Black image to create a mask
    mask = cv2.fillPoly(black_image, triangle, 255)
    # applying mask on original image
    masked_image = cv2.bitwise_and(edged,mask)
    

    #Uncomment to show canny and triangle
    # masked_image = self.bridge.cv2_to_imgmsg(masked_image, "8UC1")

    # self.mask_pub.publish(masked_image)
    # rospy.logwarn("mask published")

    #remerber edges is what we want to use

    lines = cv2.HoughLinesP(edged, 0.9, np.pi/180, 50, np.array([]), minLineLength=20, maxLineGap=600)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line.reshape(4) #converting to 1d array
            cv2.line(edged, (x1, y1), (x2, y2), (255, 0, 0), 10)

    masked_image = cv2.bitwise_and(edged,mask)
    masked_image = self.bridge.cv2_to_imgmsg(masked_image, "8UC1")
    self.mask_pub.publish(masked_image)
    # rospy.logwarn("mask published")


    image = cv2.GaussianBlur(image, (3,3), 3)
    #Dilate and Erode
    image = cv2.dilate(image, None, iterations=3)
    image = cv2.erode(image, None, iterations=3)
    # change below lines to map the color you wanted robot to follow
    lower_black = np.array([ 10,  0,  10])
    upper_black = np.array([350,55,90])
    mask = cv2.inRange(hsv, lower_black, upper_black)
    # lower_black = numpy.array([ 0,  0,  0])
    # upper_black = numpy.array([70, 70, 70])
    # mask = cv2.inRange(image, lower_black, upper_black)
 
    # speed.angular.z = -min*0.001

    # image_message = self.bridge.cv2_to_imgmsg(mask, "8UC1")
    # self.mask_pub.publish(image_message)
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = 3*h/4 + 20 
    # rospy.logwarn(search_bot)
    mask[0:search_top, 0:w] = 0
    mask[0:search_top, 0:100] = 0
    mask[0:search_top, w-100:w] = 0
    mask[0:search_top/2, 0:w] = 0

    mask[search_bot:h, 0:100] = 0
    mask[search_bot:h, w-100:w] = 0

    mask[search_bot-20:search_bot, 0:100] = 0
    mask[search_bot-20:search_bot, w-100:w] = 0
    
    
    # image_message = self.bridge.cv2_to_imgmsg(mask, "8UC1")

    # self.mask_pub.publish(image_message)

    M = cv2.moments(mask)
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    speed = Twist()
    speed.angular.z = 0
    edges = cv2.Canny(image,50,150,apertureSize=3)
    lines = cv2.HoughLines(image = edges,rho=0.1,theta=np.pi/180,threshold=100,lines=np.array([]),)
    
    # print(lines)
    # a,b,c = lines.shape
    # for i in range(a):
    #   cv2.line(image,(lines[i][0][0],lines[i][0][1]),(lines[i][0][2],lines[i][0][3]),(0,0,255),3,cv2.LINE_AA)

    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      # speed.angular.z = -err*0.001

      # self.twist.linear.x = 0.2
      # self.twist.angular.z = -float(err) / 100
      self.line_center.publish(err)
    #   self.cmd_vel_pub.publish(self.twist)
    #   # CONTROL ends
    image_message = self.bridge.cv2_to_imgmsg(image, "rgb8")
    self.image_pub.publish(image_message)
    # cv2.imshow("mask",mask)
    # cv2.imshow("output", image)

    speed.linear.x = 0.03 + speed.angular.z/3
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    posePub.publish(speed)
    cv2.waitKey(3)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
# END ALL