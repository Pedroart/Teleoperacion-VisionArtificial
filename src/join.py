#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

# Función de callback para procesar los datos recibidos
def angle_callback(data):
    # Extrae los ángulos del mensaje recibido
    angulos = data.data
    
    # Asegúrate de que los ángulos recibidos son suficientes
    if len(angulos) < 7:
        rospy.logwarn("No hay suficientes ángulos en el mensaje recibido")
        return
    
    # Mapear los ángulos a las articulaciones deseadas
    # (Este mapeo puede necesitar ajuste según el número de ángulos y su correspondencia con las articulaciones)
    print(angulos)
    jvalues = [0,0,0,0,0,0,0,0]
    jvalues[1] = angulos[0]+1.57
    jvalues[2] = angulos[1]-1.57
    jvalues[3] = angulos[2]
    jvalues[4] = 1.57*2+angulos[3]
    
    #configuracion directa

    # Crear un mensaje JointState
    jstate = JointState()
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    jstate.position = jvalues
    
    # Publicar el mensaje
    pub.publish(jstate)

if __name__ == "__main__":
    rospy.init_node("jointsNode")

    # Nombres de las articulaciones
    jnames = ("head_pan", "right_j0", "right_j1", "right_j2", "right_j3",
              "right_j4", "right_j5", "right_j6")
    
    # Crear el Publisher para joint_states
    pub = rospy.Publisher('joint_states', JointState, queue_size=10)

    # Crear el Subscriber para angle_topic
    rospy.Subscriber('angle_topic', Float64MultiArray, angle_callback)

    # Frecuencia del envío (en Hz)
    rate = rospy.Rate(100)

    # Bucle de ejecución continua
    while not rospy.is_shutdown():
        # Esperar hasta la siguiente iteración
        rate.sleep()
