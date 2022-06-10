#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Equipo 6
# Bryan Marquez - A01562119
# Ernesto Garcia Gonzalez - A00827434
# Omar Enrique Gonzalez Uresti - A00827095
# Kevin Vega Rodriguez - A01612430
# --------------------------------------------------

# Codigo de control el cual es un simple suscriptor
# al punto que envia el nodo de vision y dependiendo
# de que tan centrado este rota entre su propio eje
# y avanza al estar centrado

# NOTA
# El algoritmo de contro esta basado en pasos por el 
# callback y su correcto funcionamiento depende 
# completamente de un modulo de vision robusto

from logging import exception
import rospy
from geometry_msgs.msg import Twist, Pose, Point, TransformStamped, Vector3, Quaternion
import math
import numpy as np

#imports from socket handling

import rospy
from std_msgs.msg import String
import socket
import sys
from nav_msgs.msg import Odometry
import struct
import time
import socket, threading
import string



def callback(data):
    
    #Creamos un publisher para el control de la velocidad
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    
    mysocket = socket.socket()
    mysocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mysocket.bind(('10.42.0.1',8023))
    mysocket.listen(1)
    
    conexion,addr = mysocket.accept()
    print("Conexion establecida")
    print(addr)
    conexion.send(str(data))




def main():
    #inicializamos nuestro nodo

    rospy.init_node('Point_follower', anonymous=True)
    
   

    

    #Inicializamos nuestro subscriptor 
    rospy.Subscriber("/blob/point_blob", Point, callback)
    #Seguimos escuchando mientrar ros siga corriendo
    rospy.spin()

if __name__ == "__main__":
    # myROSconnection = toRos('10.42.0.1', 45019)
    # myROSconnection.send("bkq")
    # myROSconnection.receive()


    aux = 0
    
    main()