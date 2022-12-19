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

# This doesn't work locally
if __name__ == '__main__':
    # gather data
    cnn = CNN()
    cnn.load_model("/home/mads/project_in_robotics/project_in_robotics/vision/src/object_classification/model/retina_extraction.hdf5")
    # TODO collect some test image
    succes_count = 0
    for i in range(36):
        plug_img = cv2.imread(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/plug/{i}.jpeg")
        plug_img = plug_img[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
        plug_img = plug_img / 255.
        plug_img = np.array([plug_img])
        index = cnn.predict(plug_img)

        if index == 1:
            print("correct plug")
            succes_count = succes_count + 1
        else:
            print("wrong plug")
        screw_img = cv2.imread(f"/home/mads/project_in_robotics/project_in_robotics/vision/data/tests/rgb/screw/{i}.jpeg")
        screw_img = screw_img[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
        screw_img = screw_img / 255.
        screw_img = np.array([screw_img])
        index = cnn.predict(screw_img)
        if index == 0:
            print("correct screw")
            succes_count = succes_count + 1
        else:
            print("wrong screw")
    
    print(succes_count)