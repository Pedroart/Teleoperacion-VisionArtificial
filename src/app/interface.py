import rospy
from std_msgs.msg import String

'''
Enviador datos:
    Posicion: (array float)
    Acciones:
'''

'''
Todo:
    - Detectar si los dedos estan estirados o no
    - Detectar gestos como:
        - Dedo indice y medio como punteros
        - pausar cuando solo este el dedo indice
'''


class interface:
    def __init__(self):
        self.pub = rospy.Publisher('interface', String, queue_size=10)
        rospy.init_node('pointer', anonymous=True)
        self.rate = rospy.Rate(10)

    def run(selt):
        while not rospy.is_shutdown():
            selt.rate.sleep()


if __name__ == '__main__':
    try:
        app = interface()
        app.run()
    except rospy.ROSInterruptException:
        pass

    