import glob
import cv2

# measured manually
CROPPED_IMAGE_X1 = 65
CROPPED_IMAGE_X2 = 315
CROPPED_IMAGE_Y1 = 165
CROPPED_IMAGE_Y2 = 510


def crop_all_images():
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
            image = cv2.imread(filepath)
            image = image[CROPPED_IMAGE_X1:CROPPED_IMAGE_X2, CROPPED_IMAGE_Y1:CROPPED_IMAGE_Y2]
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
    crop_all_images()