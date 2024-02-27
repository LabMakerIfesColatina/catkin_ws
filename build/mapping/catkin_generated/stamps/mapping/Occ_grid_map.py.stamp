'''
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import random
import numpy as np
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header


roll=0.0
pitch=0.0
yaw=0.0
positionx=0.0
positiony=0.0
global occupancy
occupancy=[]

angle_increment= 0.00158543407451
angle_min = -0.513185441494
angle_max = 0.49990695715
thetasens = []


def callback1(data1):

	global roll,pitch,yaw,positionx,positiony
	quaternion =(data1.pose.pose.orientation.x, data1.pose.pose.orientation.y, data1.pose.pose.orientation.z, data1.pose.pose.orientation.w)
	euler = euler_from_quaternion(quaternion)
	roll = euler[0]
	pitch = euler[1]
	yaw = euler[2]
	positionx=data1.pose.pose.position.x
	positiony=data1.pose.pose.position.y


def callback(data):

	global ranges
	global thetasens
	print('camera')
	
	
	thetasens=np.linspace(angle_min,angle_max,640)
	ranges=[]
	for a in range(0,639):			
		ranges.append(data.ranges[a])


	mymap = OccupancyGrid(header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), 
		info = MapMetaData(width=100, height=100, resolution=0.1,map_load_time=rospy.Time.now()))
	mymap_thresh = OccupancyGrid(header = Header(seq=0, stamp = rospy.Time.now(), frame_id="map"), 
		info = MapMetaData(width=100, height=100, resolution=0.1,map_load_time=rospy.Time.now()))

	k=-2
	for i in range(0,100):
		m=-2
		for j in range(0,100):
			test = inverse_range_sensor_model(m,k,positionx,positiony,yaw,ranges)
			if test!=0:
				occupancy[i][j]=occupancy[i][j]+test
				mymap.data.append(logodds(occupancy[i][j])*100)
				if logodds(occupancy[i][j])>0.75:
					mymap_thresh.data.append(100)
				elif logodds(occupancy[i][j])<0.25:
					mymap_thresh.data.append(0)
				else:
					mymap_thresh.data.append(50)
			else:
				mymap.data.append(logodds(occupancy[i][j])*100)
				if logodds(occupancy[i][j])>0.75:
					mymap_thresh.data.append(100)
				elif logodds(occupancy[i][j])<0.25:
					mymap_thresh.data.append(0)
				else:
					mymap_thresh.data.append(50)
								
			m=m+0.1
		k=k+0.1

	publisher=rospy.Publisher('/map',OccupancyGrid,queue_size=1)
	publisher2=rospy.Publisher('/map2',OccupancyGrid,queue_size=1)
	rate = rospy.Rate(10)
	publisher.publish(mymap)
	publisher2.publish(mymap_thresh)
	
def inverse_range_sensor_model(xi,yi,x,y,theta,z):
	r=math.sqrt(((xi-x)**2)+((yi-y)**2))
	phi=math.atan2((yi-y),(xi-x))-theta	
	global angledifference	
	angledifference = []

	for i in range (0,639):
		
		angledifference.append(abs(phi-thetasens[i]))

	k=np.argmin(angledifference)

	if r > min(6,z[k]+0.25) or abs(phi-thetasens[k]) > 0.0007925 or math.isnan(z[k]):
		return 0
	if z[k]<6 and abs(r-z[k])< 0.25:
		return 1
	if r<=z[k]:
		return -1

def logodds(l):
	odds=math.exp(l)
	p=odds/(1+odds)
	return p

def main():	
	for i in range(0,100):
		occupancy.append([])
	for i in range(0,100):
		for j in range(0,100):
			occupancy[i].append(0)	

	
	while not rospy.is_shutdown():
		rospy.init_node('ListenerTalker',anonymous=False)
		rospy.Subscriber('/RosAria/pose',Odometry,callback1,queue_size=1)
		rospy.Subscriber('/scan', LaserScan,callback,queue_size=1,buff_size=2**25)
		rospy.sleep(1)
		rospy.spin()
				
'''


#!/usr/bin/env python
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
        self.depth_sub = rospy.Subscriber('/depth_vector', Float32MultiArray, self.depth_callback)
        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=10)
        self.map_metadata_pub = rospy.Publisher('/map_metadata', MapMetaData, queue_size=10)

    def depth_callback(self, data):
        try:
            # Convertendo o vetor de profundidade para uma lista
            depth_vector = data.data

            # Configurações do mapa (ajuste conforme necessário)
            resolution = 1  # Resolução do mapa em metros por pixel
            origin_x = 0    # Origem X do mapa em metros
            origin_y = 0    # Origem Y do mapa em metros

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
            rospy.logerr(e)


if __name__ == '__main__':
    try:
        depth_to_map_node = DepthToMapNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


