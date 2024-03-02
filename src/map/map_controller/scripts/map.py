

import rospy
from sensor_msgs.msg import Image ,LaserScan
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray


class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        #self.vector_pub = rospy.Publisher('/depth_vector', Float32MultiArray, queue_size=10)
        self.base_scan_pub = rospy.Publisher('/base_scan', LaserScan, queue_size=10)

    def depth_callback(self, data):
        try:
            # Converter a mensagem de imagem para matriz OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # Conversão da profundidade para metros 
            depth_in_metros = depth_image / 1000       
            linhas_desejadas = [239, 240, 241]
            
            #linhas_desejadas = [477, 478, 479]
            # Matriz para armazenar as profundidades das linhas
            matriz_profundidades = np.zeros((4, depth_in_metros.shape[1]))

            # Preencher as três primeiras linhas com as profundidades das linhas desejadas
            for i, linha in enumerate(linhas_desejadas):
                matriz_profundidades[i, :] = depth_in_metros[linha, :]
 
            
            #matriz_profundidades[:3, :]: 
            #Isso significa que estamos selecionando as três primeiras linhas da matriz matriz_profundidades. 
            #O índice :3 indica todas as linhas até (mas não incluindo) a linha de índice 3.

            #np.mean(..., axis=0): 
            #Aqui, estamos usando a função np.mean para calcular a média ao longo do eixo 0. 
            #O eixo 0 corresponde às linhas da matriz. Portanto, axis=0 indica que queremos calcular a média das colunas para cada linha. 
            #O resultado é um array que contém a média das colunas para cada posição ao longo das linhas.

            #matriz_profundidades[3, :] = ...: 
            #Finalmente, estamos atribuindo os valores calculados da média ao longo das colunas para a quarta linha da matriz matriz_profundidades.            
            matriz_profundidades[3, :] = np.mean(matriz_profundidades[:3, :], axis=0)

            # Publicar o vetor da quarta linha
            vetor_quarta_linha = matriz_profundidades[3, :]


            #self.publish_depth_vector(vetor_quarta_linha)

            laser_scan_msg = LaserScan()
            laser_scan_msg.header.stamp = rospy.Time.now()
            laser_scan_msg.header.frame_id = 'base_scan_link'  # Substitua pelo frame_id adequado

            # Configurar os parâmetros do laser scanner
            laser_scan_msg.angle_min = -1.57  # Ângulo mínimo em radianos
            laser_scan_msg.angle_max = 1.57   # Ângulo máximo em radianos
            laser_scan_msg.angle_increment = 0.0174533  # Incremento angular em radianos
            laser_scan_msg.time_increment = 0.000001  # Incremento temporal entre leituras
            laser_scan_msg.scan_time = 0.1  # Tempo total para uma varredura
            laser_scan_msg.range_min = 0.1  # Distância mínima detectável
            laser_scan_msg.range_max = 10.0  # Distância máxima detectável
            laser_scan_msg.ranges = vetor_quarta_linha.tolist()

            self.base_scan_pub.publish(laser_scan_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

    '''def publish_depth_vector(self, vetor):
        try:
            depth_vector_msg = Float32MultiArray(data=vetor.tolist())
            self.vector_pub.publish(depth_vector_msg)
        except rospy.ROSException as e:
            rospy.logerr(e)'''


def main():
    try:
        DepthConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


'''
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32MultiArray

class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.scan_pub = rospy.Publisher('/scan', LaserScan, queue_size=1)

    def depth_callback(self, data):
        try:
            # Converter a mensagem de imagem para matriz OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Simular a conversão da imagem de profundidade para leituras de varredura de laser
            scan_data = self.convert_depth_to_scan(depth_image)

            # Publicar leituras de varredura de laser no tópico /scan
            self.publish_scan(scan_data)

        except CvBridgeError as e:
            rospy.logerr(e)

    def convert_depth_to_scan(self, depth_image):
        # Lógica de conversão simulada (substitua por lógica real)
        # Aqui, estamos assumindo que cada coluna da imagem de profundidade se torna uma leitura de varredura de laser.
        # Adapte esta lógica conforme necessário para o seu sensor específico.
        scan_data = np.min(depth_image, axis=0)  # Valor mínimo ao longo das colunas

        return scan_data

    def publish_scan(self, scan_data):
        try:
            # Configurar a mensagem LaserScan
            laser_scan_msg = LaserScan()
            laser_scan_msg.header.stamp = rospy.Time.now()
            laser_scan_msg.header.frame_id = 'base_scan_link'  # Substitua pelo frame_id adequado

            # Configurar os parâmetros do laser scanner
            laser_scan_msg.angle_min = -1.57  # Ângulo mínimo em radianos
            laser_scan_msg.angle_max = 1.57   # Ângulo máximo em radianos
            laser_scan_msg.angle_increment = 0.0174533  # Incremento angular em radianos
            laser_scan_msg.time_increment = 0.000001  # Incremento temporal entre leituras
            laser_scan_msg.scan_time = 0.1  # Tempo total para uma varredura
            laser_scan_msg.range_min = 0.1  # Distância mínima detectável
            laser_scan_msg.range_max = 10.0  # Distância máxima detectável
            laser_scan_msg.ranges = scan_data.tolist()

            # Publicar a mensagem LaserScan no tópico /scan
            self.scan_pub.publish(laser_scan_msg)

        except rospy.ROSException as e:
            rospy.logerr(e)

def main():
    try:
        rospy.loginfo("Iniciando o nó Depth Converter...")
        depth_converter = DepthConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Encerrando")

if __name__ == '__main__':
    main()

'''


'''

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from std_msgs.msg import Float32MultiArray

class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.vector_pub = rospy.Publisher('/depth_vector', Float32MultiArray, queue_size=10)
    
    def depth_callback(self, data):
        try:
            # Converter a mensagem de imagem para matriz OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Conversão da profundidade para metros (anteriormente estava em milímetros)
            depth_in_metros = depth_image / 1000

            # Colunas desejadas
            colunas_desejadas = [319, 320, 321]

            # Matriz para armazenar as profundidades das colunas
            matriz_profundidades = np.zeros((depth_in_metros.shape[0], 4))

            # Preencher as três primeiras colunas com as profundidades das colunas desejadas
            for i, coluna in enumerate(colunas_desejadas):
                matriz_profundidades[:, i] = depth_in_metros[:, coluna]

            # Calcular a média ao longo das colunas
            vetor_media_horizontal = np.mean(matriz_profundidades[:, :3], axis=1)

            # Publicar o vetor da média horizontal
            self.publish_depth_vector(vetor_media_horizontal)

        except CvBridgeError as e:
            rospy.logerr(e)

    def publish_depth_vector(self, vetor):
        try:
            # Publicar o vetor como mensagem Float32MultiArray
            depth_vector_msg = Float32MultiArray(data=vetor.tolist())
            self.vector_pub.publish(depth_vector_msg)
        except rospy.ROSException as e:
            rospy.logerr(e)

def main():
    try:
        rospy.loginfo("Iniciando o nó Depth Converter...")
        depth_converter = DepthConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Encerrando")

if __name__ == '__main__':
    main()


'''



'''import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import Float32MultiArray

class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.base_scan_pub = rospy.Publisher('/base_scan', LaserScan, queue_size=10)

    def depth_callback(self, data):
        try:
            # Converter a mensagem de imagem para matriz OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')

            # Conversão da profundidade para metros (anteriormente estava em milímetros)
            depth_in_metros = depth_image / 1000

            # Linhas desejadas
            linhas_desejadas = [239, 240, 241]

            # Matriz para armazenar as profundidades das linhas
            matriz_profundidades = np.zeros((4, depth_in_metros.shape[1]))

            # Preencher as três primeiras linhas com as profundidades das linhas desejadas
            for i, linha in enumerate(linhas_desejadas):
                matriz_profundidades[i, :] = depth_in_metros[linha, :]

            # Calcular a média ao longo das linhas
            vetor_media_vertical = np.mean(matriz_profundidades[:3, :], axis=0)

            # Publicar o vetor da média vertical como mensagem LaserScan
            laser_scan_msg = LaserScan()
            laser_scan_msg.header = data.header
            laser_scan_msg.ranges = vetor_media_vertical.tolist()
            self.base_scan_pub.publish(laser_scan_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    try:
        rospy.loginfo("Iniciando o nó Depth Converter...")
        depth_converter = DepthConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Encerrando")

if __name__ == '__main__':
    main()
'''

'''
import rospy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

class DepthConverter:
    def __init__(self):
        rospy.init_node('depth_converter', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.base_scan_pub = rospy.Publisher('/minha_base', LaserScan, queue_size=10)

    def depth_callback(self, data):
        try:
            # Converter a mensagem de imagem para matriz OpenCV
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
            # Conversão da profundidade para metros 
            depth_in_metros = depth_image / 1000       
            linhas_desejadas = [239, 240, 241]
            
            #linhas_desejadas = [477, 478, 479]
            # Matriz para armazenar as profundidades das linhas
            matriz_profundidades = np.zeros((4, depth_in_metros.shape[1]))

            # Preencher as três primeiras linhas com as profundidades das linhas desejadas
            for i, linha in enumerate(linhas_desejadas):
                matriz_profundidades[i, :] = depth_in_metros[linha, :]
 
            
            #matriz_profundidades[:3, :]: 
            #Isso significa que estamos selecionando as três primeiras linhas da matriz matriz_profundidades. 
            #O índice :3 indica todas as linhas até (mas não incluindo) a linha de índice 3.

            #np.mean(..., axis=0): 
            #Aqui, estamos usando a função np.mean para calcular a média ao longo do eixo 0. 
            #O eixo 0 corresponde às linhas da matriz. Portanto, axis=0 indica que queremos calcular a média das colunas para cada linha. 
            #O resultado é um array que contém a média das colunas para cada posição ao longo das linhas.

            #matriz_profundidades[3, :] = ...: 
            #Finalmente, estamos atribuindo os valores calculados da média ao longo das colunas para a quarta linha da matriz matriz_profundidades.            
            matriz_profundidades[3, :] = np.mean(matriz_profundidades[:3, :], axis=0)

            # Publicar o vetor da quarta linha
            vetor_quarta_linha = matriz_profundidades[3, :]

            laser_scan_msg = LaserScan()
            laser_scan_msg.header = data.header
            laser_scan_msg.ranges = vetor_quarta_linha.tolist()
            self.base_scan_pub.publish(laser_scan_msg)

        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    try:
        rospy.loginfo("Iniciando o nó Depth Converter...")
        depth_converter = DepthConverter()
        rospy.spin()
    except KeyboardInterrupt:
        print("Encerrando")

if __name__ == '__main__':
    main()
'''