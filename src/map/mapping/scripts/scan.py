#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan

def scan_callback(msg):
    # Esta função será chamada sempre que uma mensagem do tipo LaserScan for recebida
    # Aqui, você pode acessar e processar os dados do scan conforme necessário
    ranges = msg.ranges
    intensities = msg.intensities
    
    # Exemplo: imprimindo os primeiros 5 valores de alcance e intensidade
    print("Ranges:", ranges[:5])
    print("Intensities:", intensities[:5])

def laser_scan_subscriber():
    # Inicializa o nó ROS com o nome "laser_scan_subscriber"
    rospy.init_node('laser_scan_subscriber', anonymous=True)

    # Subscreve no tópico "/scan" com o tipo de mensagem LaserScan
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    # Mantém o programa em execução até que seja interrompido
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_subscriber()
    except rospy.ROSInterruptException:
        pass
