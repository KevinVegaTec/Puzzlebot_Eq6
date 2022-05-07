#!/usr/bin/env python

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

import rospy
from geometry_msgs.msg import Twist, Pose, Point, TransformStamped, Vector3, Quaternion
import math
import numpy as np



def callback(data):
    #Creamos un publisher para el control de la velocidad
    posePub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)

    
    speed = Twist()
    # Usamos los umbrales para centrar lo el punto pero debido a 
    # que la deteccion no es tan rapida o perfecta necesitamos
    # que tengan una tolerancia. 
    y_th = 0.2  # En este caso podemos implementar un pequeño motor 
                # que rote la camara para centrar el objetivo
    x_th = 0.2

    #inicializamos todos los valores en 0
    speed.linear.x = 0
    speed.linear.y = 0
    speed.linear.z = 0
    speed.angular.x = 0
    speed.angular.y = 0
    speed.angular.z = 0
    alligned = False


    # Basado en el punto que recibamos siempre se busca
    # lo mas cercano a 0 entronces si el punto se encuentra
    # a la derecha giramos hacia ese sentido, de la misma 
    # manera a la izquierda.
    # La velocidad es proporcional a la distancia a la que 
    # este el punto
    if (data.x < -x_th): 
        speed.angular.z = -data.x/20
        rospy.logwarn("Turning right")

    elif (data.x > x_th):
        speed.angular.z = -data.x/20
        rospy.logwarn("Turning left")
    else: 
        speed.angular.z = 0
        # Si esta en 0 paramos y deducimos que esta alineado
        alligned = True

    #Si el puzzlebot esta alineado avanzamos (La velocidad
    # va decreciendo dependiendo de la cercania al objetivo)
    if alligned == True:
        speed.linear.x = 1/data.z
    else:
        speed.linear.x = 0
    
    # Si el area es mayor a un area determinada, el 
    # puzzlebot deja de avanzar
    if data.z > 50:
        speed.linear.x = 0
        speed.linear.y = 0
        speed.linear.z = 0
        speed.angular.x = 0
        speed.angular.y = 0
        speed.angular.z = 0

    posePub.publish(speed)

def main():
    #inicializamos nuestro nodo
    rospy.init_node('Point_follower', anonymous=True)
    
    #Inicializamos nuestro suscriptor 
    rospy.Subscriber("/blob/point_blob", Point, callback)
    #Seguimos escuchando mientrar ros siga corriendo
    rospy.spin()

if __name__ == "__main__":
    main()