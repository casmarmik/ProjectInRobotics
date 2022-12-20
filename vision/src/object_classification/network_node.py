import cv2
import rospy
from cnn import CNN

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from pir_msgs.srv import ClassifyObject
from pir_msgs.msg import PoseEstimation


# measured manually
CROPPED_IMAGE_X1 = 65
CROPPED_IMAGE_X2 = 315
CROPPED_IMAGE_Y1 = 165
CROPPED_IMAGE_Y2 = 510

class NetworkNode:
    def __init__(self, model="/home/mads/project_in_robotics/project_in_robotics/vision/src/object_classification/model/retin_extraction.hdf5"):
        self.image = None
        self.image_path = None
        self.pose_estimation_type = None
        self.take_image_flag = False

        self.cnn = CNN()
        self.cnn.load_model(model)
        
        # Setup ROS specific 
        self.sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, network_node.image_callback)
        self.ser = rospy.Service("/network_node/classify_object", ClassifyObject, network_node.capture_image)

        self.pub_pose2d = rospy.Publisher("/network_node/pose_2d", PoseEstimation)
        self.pub_pose3d = rospy.Publisher("/network_node/pose_3d", PoseEstimation)

    def classify_object(self):
        obj = self.cnn.predict(self.image)
        return obj
    
    def image_callback(self, msg):
        if self.take_image_flag:
            print("Received an image!")
            try:
                # Convert your ROS Image message to OpenCV2
                bridge = CvBridge()
                self.image = bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite(self.image_path, self.image)
                self.image = self.cnn.find_object_image(self.image_path)
                prediction = self.classify_object()
                
                msg = PoseEstimation()
                msg.filepath = self.image_path
                msg.object = prediction

                if self.pose_estimation_type == 0:
                    self.pub_pose2d.publish(msg)
                elif self.pose_estimation_type == 1:
                    self.pub_pose3d.publish(msg)
                else:
                    print("Unknown pose estimation type, please try again")

            except CvBridgeError as e:
                print(e)

            self.take_image_flag = False
    
    def capture_image(self, req):
        self.take_image_flag = True
        self.image_path = req.filepath
        self.pose_estimation_type = req.pose_estimation_type
        return True


if __name__ == '__main__':

    rospy.init_node('image_listener')

    network_node = NetworkNode()

    # Spin until ctrl + c
    rospy.spin()
