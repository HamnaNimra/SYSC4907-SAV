#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import time
import math
from typing import List, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import lane_detect


def process_image(img: np.ndarray) -> Tuple[List, float]:
    bridge = CvBridge()
    height = img.height
    width = img.width

    if img:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")

            lane_lines = lane_detect.lane_detect(cv_img)

            if lane_lines:

                # Depending on the lane lines the angle is calculated differently
                # For two lanes just take the offset from the two lanes, then take the angle to the mid point
                if len(lane_lines) == 2:
                    _, _, left_x2, _ = lane_lines[0][0]
                    _, _, right_x2, _ = lane_lines[1][0]
                    mid = int(width / 2)
                    x_offset = (left_x2 + right_x2) / 2 - mid
                    y_offset = int(height / 2)

                    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
                    steering_angle = int(angle_to_mid_radian * 180.0 / math.pi) # angle (in degrees) to center vertical line

                    return lane_lines, steering_angle, None
                # Just take the angle from the one detected lane to the midpoint
                elif len(lane_lines) == 1:
                    x1, _, x2, _ = lane_lines[0][0]
                    x_offset = x2 - x1
                    y_offset = int(height / 2)

                    angle_to_mid_radian = math.atan(x_offset / y_offset)  # angle (in radian) to center vertical line
                    steering_angle = int(angle_to_mid_radian * 180.0 / math.pi) # angle (in degrees) to center vertical line

                    return lane_lines, steering_angle, None
            else:  # An error
                print('Unable to detect lines...')
                return None, None, None

        except CvBridgeError as e:
            return None, None, e
