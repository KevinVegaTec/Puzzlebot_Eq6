#!/usr/bin/env python

# Equipo 6
# Bryan Marquez - A01562119
# Ernesto Garcia Gonzalez - A00827434
# Omar Enrique Gonzalez Uresti - A00827095
# Kevin Vega Rodriguez - A01612430
# --------------------------------------------------

# Codigo de deteccion de circulos azules el cual publica 
# el punto en el que se detecta referenciado respecto a
# el eje de coordenadas central asi como el tamano del mismo
# De manera general es un filtro HSV con otros filtros de 
# circularidad, convexidad, etc


#--- Allow relative importing
if __name__ == '__main__' and __package__ is None:
    from os import sys, path
    sys.path.append(path.dirname(path.dirname(path.abspath(__file__))))
    
import sys
# from xmlrpc.client import Boolean 3
import rospy
import cv2
import time

from std_msgs.msg           import String
from sensor_msgs.msg        import Image
from geometry_msgs.msg      import Point
from cv_bridge              import CvBridge, CvBridgeError
from include.blob_detector  import *


# Inicializamos nuestra clase blob detector (General) con algunas funciones miscelaneas
class BlobDetector:

    def __init__(self, thr_min, thr_max, blur=15, blob_params=None, detection_window=None):
    
        self.set_threshold(thr_min, thr_max)
        self.set_blur(blur)
        self.set_blob_params(blob_params)
        self.detection_window = detection_window
        
        self._t0 = time.time()
        
        self.blob_point = Point()
    
        # Se inicializan 3 publishers, el image_blob detecta el circulo
        # y lo muestra, el image_mask nos muestra la mascara resultante
        # de nuestro filtro HSV y por ultimo el point_blob publica
        # el ppunto donde se detecta un circulo azul
        print (">> Publishing image to topic image")
        self.image_pub = rospy.Publisher("/blob/image_blob",Image,queue_size=1)
        self.mask_pub = rospy.Publisher("/blob/image_mask",Image,queue_size=1)
        print (">> Publishing position to topic point_blob")
        self.blob_pub  = rospy.Publisher("/blob/point_blob",Point,queue_size=1)#---------------------------------
        self.bridge = CvBridge()
        # Suscriptor de la imagen, 
        self.image_sub = rospy.Subscriber("/video_source/raw",Image,self.callback)
        print ("<< Subscribed to topic /video_source/raw")

        
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
        # La imagen se reducio a 320x240
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
                # Se podria optimizar un algoritmo para filtrar outliers debido 
                # a que existen multiples detecciones pequenas que afectan al control
                # del puzzlebot
                x = keyPoint.pt[0]
                y = keyPoint.pt[1]
                s = keyPoint.size
                print ("kp %d: s = %3d   x = %3d  y= %3d"%(i, s, x, y))
                
                #--- Find x and y position in camera adimensional frame
                x, y = get_blob_relative_position(cv_image, keyPoint)
                
                self.blob_point.x = x
                self.blob_point.y = y
                self.blob_point.z = s
                
                self.blob_pub.publish(self.blob_point) #Publicamos el punto
                
                    
            fps = 1.0/(time.time()-self._t0)
            self._t0 = time.time()
            
def main(args):

    # Los valores cambian constantemente
    # se guardaron algunos valores para pruebas
    # sin embargo depende de la luminosidad del 
    # ambiente
    
    #blue_min = (77,40,0)
    #blue_max = (101, 255, 255) 
    #blue_min = (82,31,62)
    #blue_max = (106, 116, 193)     
    # blue_min = (112,61,129)
    # blue_max = (134, 85, 162)  
    #blue_min = (90, 60, 120)
    #blue_max = (125, 100, 155)


    blue_min = (80, 30, 40)
    blue_max = (140, 150, 140) 



    blur     = 5 # not used since we need the whole range detection

    #--- detection window respect to camera frame in [x_min, y_min, x_max, y_max] adimensional (0 to 1)
    x_min   = 0
    x_max   = 1
    y_min   = 0
    y_max   = 1
    
    detection_window = [x_min, y_min, x_max, y_max]
    
    params = cv2.SimpleBlobDetector_Params()
         
    # Change thresholds to detect the opacity 
    params.minThreshold = 20
    params.maxThreshold = 1000
     
    # Filter by Area, depends range 
    params.filterByArea = True
    params.minArea = 20
    params.maxArea = 2000000
     
    # Filter by Circularity,
    params.filterByCircularity = True
    params.minCircularity = 0.6
     
    # Filter by Convexity if the circle is complete or isn't
    params.filterByConvexity = True
    params.minConvexity = 0.6
     
    # Filter by Inertia
    params.filterByInertia = True
    params.minInertiaRatio = 0.4  

    #Creamos nuestro detector
    ic = BlobDetector(blue_min, blue_max, blur, params, detection_window)
    # Inicializamos nuestro nodo
    rospy.init_node('blob_detector', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
