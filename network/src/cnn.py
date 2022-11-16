import sys
import os
import cv2
import numpy as np

# Network stuff 
from tensorflow import keras

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

    def predict(self, img, targets):
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
        
        # Get the index for the maxium value which will give us the class
        max_value = max(prediction[0])
        index = prediction.index(max_value)
        return index 

# This doesn't work locally
if __name__ == '__main__':
    # gather data
    cnn = CNN()
    cnn.load_model("/home/mads/project_in_robotics/project_in_robotics/network/src/retina_extraction.hdf5")
    # TODO collect some test image
    test_images = cv2.imread("/home/mads/project_in_robotics/project_in_robotics/vision/data/object_classification/cropped_images/1/1.jpeg")
    test_images = test_images / 255.
    test_images = np.array([test_images])
    print(test_images.shape)
    index = cnn.predict(test_images, None)



