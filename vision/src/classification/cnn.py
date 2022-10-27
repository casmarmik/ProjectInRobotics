import glob
from pathlib import Path
import cv2

# classify data with 5s clips
none_filpath = glob.glob('vision/data/object_classification/none/*.jpeg')
skrew_filpath = glob.glob('vision/data/object_classification/skrew/*.jpeg')
stick_filpath = glob.glob('vision/data/object_classification/stick/*.jpeg')
data = []
targets = []
sr = None

none_target = 2
skrew_target = 0
stick_target = 1

cropped_image_x1 = 80
cropped_image_x2 = 360
cropped_image_y1 = 160
cropped_image_y2 = 470

for i, filepath in enumerate(none_filpath):
  with open(filepath) as file:
    image = cv2.imread(filepath)[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
    data.append(image)
    targets.append(none_target)

for i, filepath in enumerate(skrew_filpath):
  with open(filepath) as file:
    image = cv2.imread(filepath)[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
    data.append(image)
    targets.append(skrew_target)

for i, filepath in enumerate(stick_filpath):
  with open(filepath) as file:
    image = cv2.imread(filepath)[cropped_image_x1:cropped_image_x2, cropped_image_y1:cropped_image_y2]
    data.append(image)
    targets.append(stick_filpath)
  


