import glob
import cv2

# classify data with 5s clips
filepath_none = glob.glob('vision/data/object_classification/0/*.jpeg')
filepath_skrew = glob.glob('vision/data/object_classification/1/*.jpeg')
filepath_stick = glob.glob('vision/data/object_classification/2/*.jpeg')

# measured manually
cropped_image_x1 = 80
cropped_image_x2 = 360
cropped_image_y1 = 160
cropped_image_y2 = 470

image_count = 0
for i, filepath in enumerate(filepath_none):
    with open(filepath) as file:
        image = cv2.imread(filepath)
        image = image[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
        cv2.imwrite(f"vision/data/object_classification/cropped_images/0/{image_count}.jpeg", image)
        image_count = image_count + 1

image_count = 0
for i, filepath in enumerate(filepath_skrew):
    with open(filepath) as file:
        image = cv2.imread(filepath)
        image = image[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
        cv2.imwrite(f"vision/data/object_classification/cropped_images/1/{image_count}.jpeg", image)
        image_count = image_count + 1

image_count = 0
for i, filepath in enumerate(filepath_stick):
    with open(filepath) as file:
        image = cv2.imread(filepath)
        image = image[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
        cv2.imwrite(f"vision/data/object_classification/cropped_images/2/{image_count}.jpeg", image)
        image_count = image_count + 1