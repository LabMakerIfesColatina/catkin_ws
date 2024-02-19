import rospy
from std_msgs.msg import String
import serial

# Define a porta serial
serial_port = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=0.1)

def callback(msg):
    message = msg.data
    print(message)
    serial_port.write(message.encode('utf-8'))

def serial_subscriber():
    rospy.init_node('serial_subscriber', anonymous=True)
    rospy.Subscriber("/serial_command", String, callback)

    # Espera por novas mensagens
    rospy.spin()

    if serial_port.is_open:
        serial_port.close()

if __name__ == '__main__':
    try:
        serial_subscriber()
    except rospy.ROSInterruptException:
        pass
