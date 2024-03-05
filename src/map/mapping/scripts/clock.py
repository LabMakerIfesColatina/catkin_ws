#!/usr/bin/env python

import rospy
from std_msgs.msg import Clock

if __name__ == '__main__':
    rospy.init_node('clock_publisher')
    clock_pub = rospy.Publisher('/clock', Clock, queue_size=10)
    rate = rospy.Rate(1)  # Publica a cada 1 segundo (ajuste conforme necessário)

    while not rospy.is_shutdown():
        clock_msg = Clock()
        clock_msg.clock = rospy.Time.now()
        clock_pub.publish(clock_msg)
        rate.sleep()
