#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
import serial
import time

# Define a porta serial
serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)

def callback(msg):
    # Mapeia os valores dos eixos e botões para comandos
    if msg.axes[1] == 1:
        message = b'W'
    elif msg.axes[1] == -1:
        message = b'S'
    elif msg.axes[0] == 1:
        message = b'A'
    elif msg.axes[0] == -1:
        message = b'D'
    elif msg.buttons[0] == 1:
        message = b'P'
    else:
        message = b' '

    # Envia a string para a porta serial
    print(message.decode('utf-8'))
    serial_port.write(message)

# Função principal
def joystick_control():
    rospy.init_node('joystick_control', anonymous=True)
    rospy.Subscriber("/joy", Joy, callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        joystick_control()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Fecha a porta serial ao finalizar
        if serial_port.is_open:
            serial_port.close()
