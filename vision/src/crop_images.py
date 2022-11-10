import glob
import cv2

# measured manually
CROPPED_IMAGE_X1 = 80
CROPPED_IMAGE_X2 = 360
CROPPED_IMAGE_Y1 = 160
CROPPED_IMAGE_Y2 = 470


def crop_all_images():
    # classify data with 5s clips
    filepath_none = glob.glob('vision/data/object_classification/0/*.jpeg')
    filepath_skrew = glob.glob('vision/data/object_classification/1/*.jpeg')
    filepath_stick = glob.glob('vision/data/object_classification/2/*.jpeg')

    image_count = 0
    for i, filepath in enumerate(filepath_none):
        with open(filepath) as file:
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
            cv2.imwrite(f"vision/data/object_classification/cropped_images/0/{image_count}.jpeg", image)
            image_count = image_count + 1

    image_count = 0
    for i, filepath in enumerate(filepath_skrew):
        with open(filepath) as file:
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
            cv2.imwrite(f"vision/data/object_classification/cropped_images/1/{image_count}.jpeg", image)
            image_count = image_count + 1

    image_count = 0
    for i, filepath in enumerate(filepath_stick):
        with open(filepath) as file:
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
            cv2.imwrite(f"vision/data/object_classification/cropped_images/2/{image_count}.jpeg", image)
            image_count = image_count + 1

