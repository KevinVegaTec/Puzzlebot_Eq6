#!/usr/bin/env python

# Equipo 6
# Bryan Marquez - A01562119
# Ernesto Garcia Gonzalez - A00827434
# Omar Enrique Gonzalez Uresti - A00827095
# Kevin Vega Rodriguez - A01612430
# --------------------------------------------------
# CODIGO NO IMPLEMENTADO, EN REVISION

# Codigo base para deteccion de circulos rojos y 
# posterior uso en aplicacion de semaforo



#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
from xmlrpc.client import Boolean
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.blob_detector  import *


class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
    
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        print (">> Publishing image to topic image")
        self.image_pub = rospy.Publisher("/blob/image_blob_red",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/blob/image_mask_red",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/blob_dtk_red",bool,queue_size=1)#---------------------------------
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.callback)
        print ("<< Subscribed to topic /video_source/raw")
        self.blob_point.x = 0
        self.blob_point.y = 0
        self.blob_point.z = 0
        
    def set_threshold(self, thr_min, thr_max):
        self._threshold = [thr_min, thr_max]
        
    def set_blur(self, blur):
        self._blur = blur
      
    def set_blob_params(self, blob_params):
        self._blob_params = blob_params
        
    def get_blob_relative_position(self, cv_image, keyPoint):
        rows = float(cv_image.shape[0])
        cols = float(cv_image.shape[1])
        # print(rows, cols)
        center_x    = 0.5*cols
        center_y    = 0.5*rows
        # print(center_x)
        x = (keyPoint.pt[0] - center_x)/(center_x)
        y = (keyPoint.pt[1] - center_y)/(center_y)
        return(x,y)
        
        
    def callback(self,data):
        #--- Assuming image is 320x240
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (rows,cols,channels) = cv_image.shape
        if cols > 60 and rows > 60 :
            #--- Detect blobs
            keypoints, mask   = blob_detect(cv_image, self._threshold[0], self._threshold[1], self._blur,
                                            blob_params=self._blob_params, search_window=self.detection_window )
            #--- Draw search window and blobs
            cv_image    = blur_outside(cv_image, 10, self.detection_window)

            cv_image    = draw_window(cv_image, self.detection_window, line=1)
            cv_image    = draw_frame(cv_image)
            
            cv_image    = draw_keypoints(cv_image, keypoints) 
            
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, "8UC1"))
            except CvBridgeError as e:
                print(e)            

            for i, keyPoint in enumerate(keypoints):
                #--- Here you can implement some tracking algorithm to filter multiple detections
                #--- We are simply getting the first result
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                
                #--- Find x and y position in camera adimensional frame
                x, y = get_blob_relative_position(cv_image, keyPoint)
                
                # self.blob_point.x = x
                # self.blob_point.y = y
                # self.blob_point.z = s
                if ((self.blob_point.x > self.blob_point.x + 0.2) or (self.blob_point.x > self.blob_point.x -0.2)
                or  (self.blob_point.y > self.blob_point.y + 0.2) or (self.blob_point.y > self.blob_point.y - 0.2)
                or  (self.blob_point.z > self.blob_point.z + 2) or (self.blob_point.y > self.blob_point.y -2)):
                    self.blob_point.x = x
                    self.blob_point.y = y
                    self.blob_point.z = s
                    self.blob_pub.publish(self.blob_dtk_red) #----------------------------------
                else:
                    pass
                break
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            
def main(args):
    #blue_min = (77,40,0)
    #blue_max = (101, 255, 255) 
    #blue_min = (82,31,62)
    #blue_max = (106, 116, 193)     
    # blue_min = (112,61,129)
    # blue_max = (134, 85, 162)  
    red_min = (0, 70, 80)
    red_max = (20, 255, 190)


    #blue_min = (80, 20, 40)
    #blue_max = (140, 150, 140) 
       
    # red_min = (0, 70, 80)
    # red_max = (20, 255, 190)    
    



    blur     = 5
    min_size = 5
    max_size = 40
    
    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0
    x_max   = 1
    y_min   = 0
    y_max   = 1
    
    detection_window = [x_min, y_min, x_max, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds
    params.minThreshold = 20
    params.maxThreshold = 1000
     
    # Filter by Area.
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 2000000
     
    # Filter by Circularity
    params.filterByCircularity = True
    params.minCircularity = 0.6
     
    # Filter by Convexity
    params.filterByConvexity = True
    params.minConvexity = 0.6
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.4  

    ic = BlobDetector(red_min, red_max, blur, params, detection_window)
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
