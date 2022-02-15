#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import time
import math
import lane_detect
from typing import List, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


# Returns the results of both detection methods lane and approximate road boundaries
# Returns the percent difference between the two methods detection if they agree on what type of detection
# Returns the slopes of the two lines if they agree with each other, otherwise returns NAN
# Returns the current majority segment found in the segmentation image by the sum of the RGB value
def lane_interpret(img: np.ndarray, segmented_img: np.ndarray) -> Tuple[int, int, float, Tuple[float, float], int]:
    bridge = CvBridge()
    height = img.height

    if img and segmented_img:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
            cv_segmented_img = bridge.imgmsg_to_cv2(segmented_img, desired_encoding="passthrough")

            # Get a result from both the normal camera image and the segmentation image
            lane_lines, road_bounds, road_colour = lane_detect.lane_detect(cv_img, cv_segmented_img)

            # Get the lane results
            lane_result = lane_type(lane_lines, height)
            road_results = lane_type(road_bounds, height)

            # If there is two lanes
            if lane_result == 1:
                if lane_result == road_results:

                    # Get the slopes of each lane detection
                    left_lane_x1, left_lane_y1, left_lane_x2, left_lane_y2 = lane_lines[0][0]
                    right_lane_x1, right_lane_y1, right_lane_x2, right_lane_y2 = lane_lines[1][0]

                    left_road_x1, left_road_y1, left_road_x2, left_road_y2 = road_bounds[0][0]
                    right_road_x1, right_road_y1, right_road_x2, right_road_y2 = road_bounds[1][0]

                    # Get the slopes of each lane detection and the average between the two lanes
                    left_lane_slope = math.atan2((left_lane_x2 - left_lane_x1), (left_lane_y2 - left_lane_y1))
                    right_lane_slope = math.atan2((right_lane_x2 - right_lane_x1), (right_lane_y2 - right_lane_y1))
                    avg_lane_slope = (left_lane_slope + right_lane_slope) / 2

                    left_road_slope = math.atan2((left_road_x2 - left_road_x1), (left_road_y2 - left_road_y1))
                    right_road_slope = math.atan2((right_road_x2 - right_road_x1), (right_road_y2 - right_road_y1))
                    avg_road_slope = (left_road_slope + right_road_slope) / 2

                    try:
                        # Calculate the percent difference between the lane and road
                        percent_diff = abs((avg_lane_slope - avg_road_slope) / avg_road_slope)
                        return lane_result, road_results, percent_diff, (avg_lane_slope, avg_road_slope), road_colour
                    # One of the slopes could be zero or close to it,
                    except ZeroDivisionError as e:
                        return lane_result, road_results, 1, (avg_lane_slope, avg_road_slope), road_colour
                else:
                    return lane_result, road_results, 1, ('NAN', 'NAN'), road_colour

            # If there is only one lane
            elif lane_result == 2 or lane_result == 3:
                if lane_result == road_results:
                    lane_x1, lane_y1, lane_x2, lane_y2 = lane_lines[0][0]
                    lane_slope = math.atan2((lane_x2 - lane_x1), (lane_y2 - lane_y1))

                    road_x1, road_y1, road_x2, road_y2 = road_bounds[0][0]
                    road_slope = math.atan2((road_x2 - road_x1), (road_y2 - road_y1))

                    try:
                        percent_diff = abs((lane_slope - road_slope) / road_slope)
                        return lane_result, road_results, percent_diff, (lane_slope, road_slope), road_colour
                    except ZeroDivisionError as e:
                        return lane_result, road_results, 1, (avg_lane_slope, avg_road_slope), road_colour

                # No lanes detected
                else:
                    return lane_result, road_results, 1, ('NAN', 'NAN'), road_colour
            else:
                return lane_result, road_results, 0, ('NAN', 'NAN'), road_colour

        except CvBridgeError as e:
            return 4, 4, 0, ('NAN', 'NAN'), 0


def lane_type(lines, height):
    if lines:
        if len(lines) == 2:
            return 1
        elif len(lines) == 1:
            x1, _, x2, _ = lines[0][0]
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
