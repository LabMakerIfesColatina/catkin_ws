#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from std_msgs.msg import Header

def publish_static_odom():
    rospy.init_node('static_odom_publisher', anonymous=True)
    odom_pub = rospy.Publisher('odom_frame', Odometry, queue_size=10)
    rate = rospy.Rate(1)  # Publica a cada 1 segundo (ajuste conforme necessário)

    while not rospy.is_shutdown():
        # Criar uma mensagem de odometria com os valores desejados
        odom_msg = Odometry()
        odom_msg.header = Header()
        odom_msg.header.stamp = rospy.Time.now()
        odom_msg.header.frame_id = "odom"
        odom_msg.child_frame_id = "base_link"
        
        # Configurar a posição estática (ajuste conforme necessário)
        odom_msg.pose.pose = Pose(Point(0, 0, 0), Quaternion(0, 0, 0, 1))
        
        # Configurar a velocidade (opcional)
        odom_msg.twist.twist = Twist()

        # Publicar a mensagem
        odom_pub.publish(odom_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_static_odom()
    except rospy.ROSInterruptException:
        pass
