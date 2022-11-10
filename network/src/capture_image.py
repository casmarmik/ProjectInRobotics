import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

# Customized srv to capture  
from network.srv import image_filepath

take_image_flag = False
filepath = ""

def capture_image(req):
    global filepath, take_image_flag
    filepath = req.filepath
    take_image_flag = True

def image_callback(msg):
    global filepath
    global take_image_flag
    if take_image_flag:
        take_image_flag = False
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            bridge = CvBridge()
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        cv2.imwrite(filepath, cv2_img)

if __name__ == '__main__':
    rospy.init_node('image_listener')

    # Set up your subscriber and define its callback
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback)
    s = rospy.Service("/image_capture", image_filepath, capture_image)
    # Spin until ctrl + c
    rospy.spin()

