# Code is taking from here https://stackoverflow.com/questions/49333629/opencv-detecting-objects-excluding-shadows
"""
control_bar_hsv.py
"""

import cv2
from pathlib import Path
import numpy as np
import ctypes
import functools
from typing import Tuple, Callable


class ControlBarBase:
    __slots__ = ('img_bgr',)
    WAIT_TIME = 500  # milliseconds
    CONTROL_PANEL_NAME = 'control_panel'
    IMAGE_PANEL_NAME = 'image'
    #SCREEN_SIZE: Tuple[int, int] = None

    def __init__(self, img_path: Path):
        self.img_bgr: np.ndarray = cv2.imread(str(img_path))
        #self.img_bgr = self.img_bgr[77:300, 207:458]
        #self.img_bgr = self.img_bgr[36:1937, 577:2120] # change when the setup is changed
        self.init_window()
        self.init_track_bar()

    def init_window(self):
        for name in (self.CONTROL_PANEL_NAME, self.IMAGE_PANEL_NAME):
            cv2.namedWindow(name, cv2.WINDOW_NORMAL)
            #if self.SCREEN_SIZE:
            #    screen_width, screen_height = self.SCREEN_SIZE
            #    cv2.resizeWindow(name, int(screen_width), int(screen_height))

    def init_track_bar(self):
        """
        self.build_track_bar(label_name, range_lim=(0, 255), default_value=0, callback)
        """
        raise NotImplementedError('subclasses of _ControlBarBase must provide a init_track_bar() method')

    def render(self):
        raise NotImplementedError('subclasses of _ControlBarBase must provide a render() method.')

    def build_track_bar(self, label_name: str,
                        range_lim: Tuple[int, int], default_value: int, callback: Callable = lambda x: ...):
        min_val, max_val = range_lim
        cv2.createTrackbar(label_name, self.CONTROL_PANEL_NAME, min_val, max_val, callback)
        cv2.setTrackbarPos(label_name, self.CONTROL_PANEL_NAME, default_value)

    def get_trackbar_pos(self, widget_name: str):
        return cv2.getTrackbarPos(widget_name, self.CONTROL_PANEL_NAME)

    def run(self):
        while 1:
            img, callback_func = self.render()
            cv2.imshow(self.IMAGE_PANEL_NAME, img)
            if (cv2.waitKey(self.WAIT_TIME) & 0xFF == ord('q') or
                    cv2.getWindowProperty(self.IMAGE_PANEL_NAME, 0) == -1 or
                    cv2.getWindowProperty(self.CONTROL_PANEL_NAME, 0) == -1):
                callback_func()
                break
        cv2.destroyAllWindows()


class ControlBarHSV(ControlBarBase):
    __slots__ = ()
    WAIT_TIME = 500
    #SCREEN_SIZE = ctypes.windll.user32.GetSystemMetrics(0) / 2, ctypes.windll.user32.GetSystemMetrics(1) / 2

    def init_track_bar(self):
        self.build_track_bar('HMin', range_lim=(0, 179), default_value=150)
        self.build_track_bar('SMin', (0, 255), 55)
        self.build_track_bar('VMin', (0, 255), 50)

        self.build_track_bar('HMax', (0, 179), 179)
        self.build_track_bar('SMax', (0, 255), 120)
        self.build_track_bar('VMax', (0, 255), 158)

    def render(self):
        # get current positions of all trackbars
        h_min = self.get_trackbar_pos('HMin')
        s_min = self.get_trackbar_pos('SMin')
        v_min = self.get_trackbar_pos('VMin')

        h_max = self.get_trackbar_pos('HMax')
        s_max = self.get_trackbar_pos('SMax')
        v_max = self.get_trackbar_pos('VMax')

        # Set minimum and max HSV values to display
        lower = np.array([h_min, s_min, v_min])
        upper = np.array([h_max, s_max, v_max])

        # Create HSV Image and threshold into a range.
        hsv = cv2.cvtColor(self.img_bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower, upper)
        img_output: np.ndarray = cv2.bitwise_and(self.img_bgr, self.img_bgr, mask=mask)

        @functools.wraps(self.render)
        def cb_func():
            return print(f'cv2.inRange(src=hsv, lowerb=np.array([{h_min}, {s_min}, {v_min}]), upperb=np.array([{h_max}, {s_max}, {v_max}]))')

        return img_output, cb_func


if __name__ == '__main__':
    image_path = "vision/data/object_classification_data/cropped_images2/1/1.jpeg"
    obj = ControlBarHSV(image_path)
    obj.run()