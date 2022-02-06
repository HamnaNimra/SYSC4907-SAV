#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import time
import math
from typing import List, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import lane_detect


def process_image(img: np.ndarray) -> int:
    bridge = CvBridge()
    height = img.height

    if img:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

            lane_lines = lane_detect.lane_detect(cv_img)

            if lane_lines:
                if len(lane_lines) == 2:
                    return 1
                elif len(lane_lines) == 1:
                    x1, _, x2, _ = lane_lines[0][0]
                    x_offset = x2 - x1
                    y_offset = int(height / 2)
                    angle_to_mid_radian = math.atan(x_offset / y_offset)
                    steering_angle = int(angle_to_mid_radian * 180.0 / math.pi)

                    if steering_angle < 0:
                        return 2
                    if steering_angle > 0:
                        return 3
            else:
                return 4

        except CvBridgeError as e:
            return 4
