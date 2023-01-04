import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from pir_msgs.srv import ClassifyObject
from pir_msgs.msg import PoseEstimation

import numpy as np

# Network stuff 
from tensorflow import keras

# measured manually
CROPPED_IMAGE_X1 = 65
CROPPED_IMAGE_X2 = 315
CROPPED_IMAGE_Y1 = 165
CROPPED_IMAGE_Y2 = 510

class CNN():
    def __init__(self):
        self.model = None
        self.lower = (34, 0, 0)
        self.upper= (179, 255, 180)

        # Measured manually 
        self.CROPPED_IMAGE2_X1 = 77
        self.CROPPED_IMAGE2_X2 = 300
        self.CROPPED_IMAGE2_Y1 = 207
        self.CROPPED_IMAGE2_Y2 = 458


    def load_model(self, model_path):
        """
        Loads the CNN model, which has been pretrained on colab

        Parameters
        ----------
        model_path (string): The path to the stored model
        """
        self.model = keras.models.load_model(model_path)

    def predict(self, img):
        """
        Predict on the given data

        Parameters
        ----------
        Test data: test data
        test_targets: test targets
        """
        if self.model == None:
            print("Load model before you can predict")
            return 
        
        # Maybe it would make sense to measure how much time the prediction takes
        prediction = self.model.predict(img)
        
        print(prediction[0])
        # Get the index for the maxium value which will give us the class
        index = np.argmax(prediction[0])
        return index 

    def find_object_image(self, filepath):
        orig_image = cv2.imread(filepath)
        image = np.zeros([orig_image.shape[0], orig_image.shape[1], 3], dtype=np.uint8)
        for i in range(self.CROPPED_IMAGE2_X1, self.CROPPED_IMAGE2_X2):
            for j in range(self.CROPPED_IMAGE2_Y1, self.CROPPED_IMAGE2_Y2):
                image[i,j, 0] = orig_image[i,j, 0]
                image[i,j, 1] = orig_image[i,j, 1]
                image[i,j, 2] = orig_image[i,j, 2]

        mask = cv2.inRange(image, self.lower, self.upper)
        masked_image = cv2.bitwise_and(image, image, mask=mask)

        gray_image = cv2.cvtColor(masked_image, cv2.COLOR_BGR2GRAY)
        gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        gray_image = cv2.GaussianBlur(gray_image, (5, 5), 0)
        _, binary_image = cv2.threshold(gray_image, 60, 100, cv2.THRESH_BINARY)
        kernel = np.ones((3, 3), np.uint8)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)
        binary_image = cv2.dilate(binary_image, kernel, iterations=1)

        contours, _ = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour = None
        if len(contours) > 1:
            max_area = -100
            max_area_idx = -1
            for i in range(len(contours)):
                area = cv2.contourArea(contours[i])
                if area > max_area:
                    max_area = area
                    max_area_idx = i
            
            contour = contours[max_area_idx]

        elif len(contours) < 1:
            print("failied to find contour")
            return orig_image
        else:
            contour = contours[0]

        x,y,w,h =  cv2.boundingRect(contour)
        center_x = int(x + (w/2))
        center_y = int(y + (h/2))
        x = center_x - 50
        y = center_y - 50
        w = 100
        h = 100
        classification_image = orig_image[y:y+h, x:x+w]
        return classification_image

class NetworkNode:
    def __init__(self, model="/home/mads/project_in_robotics/project_in_robotics/vision/src/object_classification/model/retin_extraction.hdf5"):
        self.image = None
        self.image_path = None
        self.pose_estimation_type = None
        self.take_image_flag = False

        self.cnn = CNN()
        self.cnn.load_model(model)
        
        # Setup ROS specific 
        self.sub = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.image_callback)
        self.ser = rospy.Service("/network_node/classify_object", ClassifyObject, self.capture_image)

        self.pub_pose2d = rospy.Publisher("/network_node/pose_2d", PoseEstimation, queue_size=100)
        self.pub_pose3d = rospy.Publisher("/network_node/pose_3d", PoseEstimation, queue_size=100)

    def classify_object(self):
        img =self.image / 255.
        img = np.array([img])
        obj = self.cnn.predict(img)
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
                print(prediction)

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
