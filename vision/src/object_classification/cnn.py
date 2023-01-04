import sys
import os
import cv2
from cv2 import imshow
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


# This doesn't work locally
if __name__ == '__main__':
    # gather data
    cnn = CNN()
    cnn.load_model("/home/mads/project_in_robotics/project_in_robotics/vision/src/object_classification/model/retin_extraction.hdf5")
    # TODO collect some test image
    succes_count = 0
    plug_count = 0
    for i in range(36):
        plug_img = cnn.find_object_image(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/plug/{i}.jpeg")
        plug_img = plug_img / 255.
        plug_img = np.array([plug_img])
        index = cnn.predict(plug_img)
        
        if index == 1:
            #print("correct plug")
            succes_count = succes_count + 1
        else:
            print(f"wrong plug {plug_count}")
        plug_count = plug_count + 1
        screw_img = cnn.find_object_image(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/screw/{i}.jpeg")
        screw_img = screw_img / 255.
        screw_img = np.array([screw_img])
        index = cnn.predict(screw_img)
        if index == 0:
            #print("correct screw")
            succes_count = succes_count + 1
        else:
            print(f"wrong screw {succes_count}")
    
    print(succes_count)