#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def callback_pointer(data):
    """
    Callback function that gets called whenever a message is received on the /pointer topic.
    Publishes the received data to the /xyz_target topic.
    """
    global xyz_target_pub
    modified_data = Point()
    modified_data.x = -data.x  # Equivalente a no cambiar el valor
    modified_data.y = data.y - 0.4
    modified_data.z = 0.2  # Establecer z a un valor fijo
    xyz_target_pub.publish(modified_data)

def main():
    rospy.init_node('pointer_to_xyz_target', anonymous=True)

    # Subscriber to the /pointer topic
    rospy.Subscriber('/pointer', Point, callback_pointer)

    # Publisher for the /xyz_target topic
    global xyz_target_pub
    xyz_target_pub = rospy.Publisher('/xyz_target', Point, queue_size=10)

    rospy.loginfo("Node pointer_to_xyz_target started, relaying messages from /pointer to /xyz_target")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
