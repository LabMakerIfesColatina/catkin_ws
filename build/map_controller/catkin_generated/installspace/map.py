

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
            self.publish_depth_vector(vetor_quarta_linha)

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