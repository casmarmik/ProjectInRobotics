import glob
import cv2
import numpy as np
from pandas import Int32Dtype

# measured manually
CROPPED_IMAGE_X1 = 65
CROPPED_IMAGE_X2 = 315
CROPPED_IMAGE_Y1 = 165
CROPPED_IMAGE_Y2 = 510

# Measured manually 
CROPPED_IMAGE2_X1 = 77
CROPPED_IMAGE2_X2 = 300
CROPPED_IMAGE2_Y1 = 207
CROPPED_IMAGE2_Y2 = 458


lower = (34, 0, 0)
upper_screw = (179, 255, 160)
upper_plug = (179, 255, 180)

def find_all_objects():
    # classify data with 5s clips
    #filepath_none = glob.glob('vision/data/object_classification_data/0/*.jpeg')
    filepath_skrew = glob.glob('vision/data/object_classification_data/1/*.jpeg')
    filepath_stick = glob.glob('vision/data/object_classification_data/2/*.jpeg')

    # image_count = 0
    # for i, filepath in enumerate(filepath_none):
    #     with open(filepath) as file:
    #         image = cv2.imread(filepath)
    #         image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
    #         cv2.imwrite(f"vision/data/object_classification_data/cropped_images2/0/{image_count}.jpeg", image)
    #         image_count = image_count + 1

    image_count = 0
    for i, filepath in enumerate(filepath_skrew):
        with open(filepath) as file:
            orig_image = cv2.imread(filepath)
            image = np.zeros([orig_image.shape[0], orig_image.shape[1], 3], dtype=np.uint8)
            for i in range(CROPPED_IMAGE2_X1, CROPPED_IMAGE2_X2):
                for j in range(CROPPED_IMAGE2_Y1, CROPPED_IMAGE2_Y2):
                    image[i,j, 0] = orig_image[i,j, 0]
                    image[i,j, 1] = orig_image[i,j, 1]
                    image[i,j, 2] = orig_image[i,j, 2]

            mask = cv2.inRange(image, lower, upper_plug)
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

            contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                continue
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
            cv2.imwrite(f"vision/data/object_classification_data/objects_images/0/{image_count}.jpeg", classification_image)
            image_count = image_count + 1

    image_count = 0
    for i, filepath in enumerate(filepath_stick):
        with open(filepath) as file:
            orig_image = cv2.imread(filepath)
            image = np.zeros([orig_image.shape[0], orig_image.shape[1], 3], dtype=np.uint8)
            for i in range(CROPPED_IMAGE2_X1, CROPPED_IMAGE2_X2):
                for j in range(CROPPED_IMAGE2_Y1, CROPPED_IMAGE2_Y2):
                    image[i,j, 0] = orig_image[i,j, 0]
                    image[i,j, 1] = orig_image[i,j, 1]
                    image[i,j, 2] = orig_image[i,j, 2]
            mask = cv2.inRange(image, lower, upper_plug)
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

            contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
                continue
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
            cv2.imwrite(f"vision/data/object_classification_data/objects_images/1/{image_count}.jpeg", classification_image)
            image_count = image_count + 1

def crop_all_images():

    filepath_skrew = glob.glob('vision/data/object_classification_data/1/*.jpeg')
    filepath_stick = glob.glob('vision/data/object_classification_data/2/*.jpeg')

    image_count = 0
    for i, filepath in enumerate(filepath_skrew):
        with open(filepath) as file:
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE2_X1:CROPPED_IMAGE2_X2, CROPPED_IMAGE2_Y1:CROPPED_IMAGE2_Y2]
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            cv2.imwrite(f"vision/data/object_classification_data/cropped_images2/1/{image_count}.jpeg", image)
            image_count = image_count + 1

    image_count = 0
    for i, filepath in enumerate(filepath_stick):
        with open(filepath) as file:
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
            cv2.imwrite(f"vision/data/object_classification_data/cropped_images2/2/{image_count}.jpeg", image)
            image_count = image_count + 1

if __name__ == '__main__':
    find_all_objects()
    #img = cv2.imread("vision/data/object_classification_data/1/1.jpeg")
    #img = img[CROPPED_IMAGE2_X1:CROPPED_IMAGE2_X2, CROPPED_IMAGE2_Y1:CROPPED_IMAGE2_Y2]
    #cv2.imshow("cropped immage", img)
    #cv2.waitKey()