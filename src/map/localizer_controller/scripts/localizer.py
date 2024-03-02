
import rospy
from std_msgs.msg import String

def publicar_string():
    rospy.init_node('publicador_string', anonymous=True)
    rate = rospy.Rate(1)  # 1 Hz

    string_pub = rospy.Publisher('serial_command', String, queue_size=10)

    while not rospy.is_shutdown():
        mensagem = "0,0"
        string_pub.publish(mensagem)
        rate.sleep()

if __name__ == '__main__':
    try:
        publicar_string()
    except rospy.ROSInterruptException:
        pass
