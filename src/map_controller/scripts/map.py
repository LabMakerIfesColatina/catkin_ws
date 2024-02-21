import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class KinectImageViewer:
    def __init__(self):
        rospy.init_node('kinect_image_viewer', anonymous=True)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
            cv2.imshow('Kinect Image', cv_image)
            cv2.waitKey(1)
        except CvBridgeError as e:
            rospy.logerr(e)

def main():
    try:
        image_viewer = KinectImageViewer()
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()