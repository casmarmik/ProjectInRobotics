#!/usr/bin/env python3

# rospy for the subscriber
import rospy
# ROS Image message
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
# OpenCV2 for saving an image
import cv2

from pir_msgs.srv import image_capture

# Instantiate CvBridge
bridge = CvBridge()

# Global flag of when to take an image
take_image_flag = False
none_counter = 0
multiple_counter = 0
skrew_counter = 120
stick_counter = 120
calibration_counter = 0
target = -1
  
def capture_image(req):
    global target

    if req.target == 0:
        target = "multiple"
    elif req.target == 1:
        target = "none"
    elif req.target == 2:
        target = "skrew"
    elif req.target == 3:
            target = "stick"
    elif req.target == 4:
        target = "calibration"
    else:
        print("unknown input")
        return
      
    global take_image_flag
    take_image_flag = True

def image_callback(msg):
    global target
    global take_image_flag
    if take_image_flag:
        take_image_flag = False
        print("Received an image!")
        try:
            # Convert your ROS Image message to OpenCV2
            cv2_img = bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        # Save your OpenCV2 image as a jpeg 
        if target == "none":
            global none_counter
            cv2.imwrite(f"/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/tests/rgb/screw/{none_counter}.jpeg", cv2_img)
            none_counter = none_counter + 1
        elif target == "multiple":
            global multiple_counter
            cv2.imwrite(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/object_classification_data/{target}/{multiple_counter}.jpeg", cv2_img)
        elif target == "skrew":
            global skrew_counter
            cv2.imwrite(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/object_classification_data/1/{skrew_counter}.jpeg", cv2_img)
            skrew_counter = skrew_counter + 1
        elif target == "stick":
            global stick_counter
            cv2.imwrite(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/object_classification_data/2/{stick_counter}.jpeg", cv2_img)
            stick_counter = stick_counter + 1
        elif target == "calibration":
            global calibration_counter
            print("saving image")
            cv2.imwrite(f"/home/marcus/pir/ros_ws/src/project_in_robotics/vision/data/calibration/{calibration_counter}.jpeg", cv2_img)
            calibration_counter = calibration_counter + 1

        else:
            print("unknown target")


def main():
    rospy.init_node('image_listener')

    # Set up your subscriber and define its callback
    rospy.Subscriber("/camera/rgb/image_rect_color", Image, image_callback)
    s = rospy.Service("/image_capture", image_capture, capture_image)
    # Spin until ctrl + c
    rospy.spin()

if __name__ == '__main__':
    main()