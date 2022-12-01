import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from cnn import CNN
from std_msgs.msg import String

# measured manually
CROPPED_IMAGE_X1 = 80
CROPPED_IMAGE_X2 = 360
CROPPED_IMAGE_Y1 = 160
CROPPED_IMAGE_Y2 = 470

class NetworkNode:
    def __init__(self, model="/home/mads/project_in_robotics/project_in_robotics/network/model/retina_extraction.hdf5"):
        self.image = None
        self.take_image_flag = False
        self.cnn = CNN()
        self.cnn.load_model(model)

    def crop_single_image(self):
        if self.image == None:
            print("no image has been taking cannot crop image")
        self.image = self.image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
    
    def image_callback(self, msg):
        if self.take_image_flag:
            print("Received an image!")
            try:
                # Convert your ROS Image message to OpenCV2
                bridge = CvBridge()
                self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
                self.crop_single_image()
            except CvBridgeError as e:
                print(e)

            self.take_image_flag = False
    
    def capture_image(self, req):
        self.take_image_flag = True


if __name__ == '__main__':

    rospy.init_node('image_listener')

    network_node = NetworkNode()

	# TODO make a service that is called to start everything
    # Set up your subscriber and define its callback
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, network_node.image_callback)
    s = rospy.Service("/classify_object", String, network_node.capture_image)
    # Spin until ctrl + c
    rospy.spin()
