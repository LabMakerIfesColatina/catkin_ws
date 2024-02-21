import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import socket
import pickle
import struct

def image_subscriber():
    rospy.init_node('kinect_image_subscriber', anonymous=True)

    # Inicializa o OpenCV para exibir a imagem recebida
    cv2.namedWindow("Received Image", cv2.WINDOW_NORMAL)

    # Configuração da conexão TCP
    host = ''  # Escuta em todas as interfaces
    port = 12345  # Substitua pela porta usada pelo nodo de transmissão
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)

    # Aguarda a conexão do nodo de transmissão
    print(f"Aguardando conexão na porta {port}...")
    client_socket, addr = server_socket.accept()
    print(f"Conexão estabelecida de {addr}")

    # Inicializa o publicador de imagem
    image_pub = rospy.Publisher('kinect_image_topic', Image, queue_size=10)
    bridge = CvBridge()

    while not rospy.is_shutdown():
        # Recebe o tamanho dos dados
        size_data = client_socket.recv(4)
        size = struct.unpack("!I", size_data)[0]

        # Recebe os dados da imagem
        image_data = b""
        while len(image_data) < size:
            image_data += client_socket.recv(size - len(image_data))

        # Deserializa os dados da imagem
        image_msg = pickle.loads(image_data)

        # Publica a imagem no tópico
        image_pub.publish(image_msg)

        # Converte a imagem para array OpenCV e exibe
        cv_image = bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)

    # Fecha a conexão TCP quando o nó é encerrado
    client_socket.close()
    server_socket.close()

if __name__ == '__main__':
    try:
        image_subscriber()
    except rospy.ROSInterruptException:
        pass