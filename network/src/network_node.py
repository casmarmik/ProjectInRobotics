from cnn import CNN
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError

# measured manually
CROPPED_IMAGE_X1 = 80
CROPPED_IMAGE_X2 = 360
CROPPED_IMAGE_Y1 = 160
CROPPED_IMAGE_Y2 = 470

def crop_single_image(image_path, cropped_image_path):
    image = cv2.imread(image_path)
    image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
    cv2.imwrite(f"{cropped_image_path}", image)

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

def classify_object(req):
    global filepath, take_image_flag
    filepath = req.filepath
    take_image_flag = True

if __name__ == '__main__':

    rospy.init_node('image_listener')

	# TODO make a service that is called to start everything
    # Set up your subscriber and define its callback
    #rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback)
    #s = rospy.Service("/classify_object", image_filepath, capture_image)
    # Spin until ctrl + c
    rospy.spin()
