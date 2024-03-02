

'''import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np
class DepthToMapNode:
    def __init__(self):
        rospy.init_node('depth_to_map_node', anonymous=True)
        self.bridge = CvBridge()
        self.depth_sub = rospy.Subscriber('/depth_vector', Float32MultiArray, self.depth_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.map_metadata_pub = rospy.Publisher('/map_metadata', MapMetaData, queue_size=10)

    def depth_callback(self, data):
        try:
            # Convertendo o vetor de profundidade para uma lista
            depth_vector = data.data

            
            resolution = 1  # Resolução do mapa em metros por pixel
            origin_x = -640/2   # Origem X do mapa em metros
            origin_y = 0.0    # Origem Y do mapa em metros

            # Criando a mensagem de metadados do mapa
            map_metadata = MapMetaData()
            map_metadata.resolution = resolution
            map_metadata.width = len(depth_vector)
            map_metadata.height = 1  # Apenas uma linha
            map_metadata.origin.position.x = origin_x
            map_metadata.origin.position.y = origin_y

            # Publicando a mensagem de metadados do mapa
            self.map_metadata_pub.publish(map_metadata)

            # Criando a mensagem de ocupação do mapa
            map_data = OccupancyGrid()
            map_data.header.stamp = rospy.Time.now()
            map_data.header.frame_id = 'map'
            map_data.info = map_metadata

            # Convertendo as profundidades para dados de ocupação
            occupancy_data = (np.array(depth_vector) < 1.0).astype(np.int8) * 100
            map_data.data = occupancy_data.tolist()



            # Publicando a mensagem de ocupação do mapa
            self.map_pub.publish(map_data)

        except Exception as e:
            rospy.logerr(e)'''


import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import OccupancyGrid, MapMetaData
import numpy as np

class DepthToMapNode:
    def __init__(self):
        rospy.init_node('depth_to_map_node', anonymous=True)
        self.bridge = CvBridge()
        self.map_pub = rospy.Publisher('/base_scans', OccupancyGrid, queue_size=10)
        self.map_metadata_pub = rospy.Publisher('/map_metadata', MapMetaData, queue_size=10)
        self.depth_sub = rospy.Subscriber('/depth_vector', Float32MultiArray, self.depth_callback)


        self.resolution = 0.1  # Resolução do mapa em metros por pixel
        self.origin_x = -50.0   # Origem X do mapa em metros
        self.origin_y = 0.0   # Origem Y do mapa em metros

    def depth_callback(self, data):
        try:
            # Convertendo o vetor de profundidade para uma lista
            depth_vector = data.data

            # Criando a mensagem de metadados do mapa
            map_metadata = MapMetaData()
            map_metadata.resolution = self.resolution
            map_metadata.width = len(depth_vector)
            map_metadata.height = 1 
            map_metadata.origin.position.x = self.origin_x
            map_metadata.origin.position.y = self.origin_y
            self.map_metadata_pub.publish(map_metadata)

            # Criando a mensagem de ocupação do mapa
            map_data = OccupancyGrid()
            map_data.header.stamp = rospy.Time.now()
            map_data.header.frame_id = 'base_scans'
            map_data.info = map_metadata
            occupancy_data = (np.array(depth_vector) < 1.0).astype(np.int8) * 100
            map_data.data = occupancy_data.tolist()
            self.map_pub.publish(map_data)

        except Exception as e:
            rospy.logerr(e)


def main():
    try:
        DepthToMapNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
    

