import math
import time
from copy import copy
from typing import List, Dict
import rospy
from std_msgs.msg import String

serial_command_pub = rospy.Publisher("/serial_command", String, queue_size=10)

class Movimento:
    
    @staticmethod
    def frente():
        serial_command_pub.publish('W')
        rospy.loginfo('W')
        time.sleep(1.8)

    @staticmethod
    def virar90direita():
        serial_command_pub.publish('D')
        rospy.loginfo('D')
        time.sleep(1.2)

    @staticmethod
    def virar90esquerda():
        serial_command_pub.publish('A')
        rospy.loginfo('A')
        time.sleep(1.15)

    @staticmethod
    def virar180():
        serial_command_pub.publish('D')
        time.sleep(2.4)
        rospy.loginfo('S')

    @staticmethod
    def parar():
        serial_command_pub.publish('P')
        rospy.loginfo('P')

def main():
    time.sleep(5)
    rospy.init_node("controlador")

    Movimento.virar90esquerda()
    Movimento.parar()
    
    '''Movimento.virar90direita()
    Movimento.parar()
    Movimento.virar90esquerda()
    Movimento.parar()
    Movimento.frente()
    Movimento.parar()'''

   

if __name__ == '__main__':
    main()