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
# Returns the coordinates of the two lines
# Returns the current majority segment found in the segmentation image by the sum of the RGB value
def lane_interpret(img: np.ndarray, segmented_img: np.ndarray):
    bridge = CvBridge()
    height = img.height

    if img and segmented_img:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
            cv_segmented_img = bridge.imgmsg_to_cv2(segmented_img, desired_encoding="passthrough")

            # Get a result from both the normal camera image and the segmentation image
            gradient_lanes, hls_lanes, segmentation_lanes, road_colour = lane_detect.lane_detect(cv_img, cv_segmented_img)

            # Get the lane results
            gradient_result = lane_type(gradient_lanes, height)
            hls_result = lane_type(hls_lanes, height)
            segmentation_result = lane_type(segmentation_lanes, height)

            # Get the percent differences for the detections
            gradient_difference = compare_lanes(gradient_lanes, segmentation_lanes, gradient_result,
                                                segmentation_result)
            hls_difference = compare_lanes(hls_lanes, segmentation_lanes, hls_result, segmentation_result)

            return ((gradient_result, hls_result, segmentation_result), (gradient_difference, hls_difference),
                    (format_lanes(gradient_lanes), format_lanes(hls_lanes), format_lanes(segmentation_lanes)), road_colour)

        except CvBridgeError as e:
            return (4, 4, 4), (100, 100), ([], [], []), 1


# Returns the type of lane detection
# 1 is two lanes
# 2 is one lane off to the left
# 3 is one lane off to the right
# 4 is no lanes
def lane_type(lines, height):
    if lines:
        if len(lines) == 2:
            return 1
        elif len(lines) == 1:
            x1, y1, x2, y2 = lines[0]
            slope = (y2-y1) / (x2-x1)

            if slope < 0:
                return 2
            if slope > 0:
                return 3
    else:
        return 4


# Get the percentage difference between the two lane detections
# lanes_1 and lanes_2 are both detections, lanes_2 is a "ground truth"
def compare_lanes(lanes_1, lanes_2, lane_detection1, lane_detection2) -> List[float]:
    # If there is two lanes
    if lane_detection1 == 1:
        # If the lanes are the same get the percent difference
        if lane_detection1 == lane_detection2:

            # Get the slopes of each lane detection
            left_lane1_x1, left_lane1_y1, left_lane1_x2, left_lane1_y2 = lanes_1[0]
            right_lane1_x1, right_lane1_y1, right_lane1_x2, right_lane1_y2 = lanes_1[1]

            left_lane2_x1, left_lane2_y1, left_lane2_x2, left_lane2_y2 = lanes_2[0]
            right_lane2_x1, right_lan2_y1, right_lane2_x2, right_lane2_y2 = lanes_2[1]

            # Get the slopes of each lane detection and the average between the two lanes
            left_lane1_slope = (left_lane1_y2 - left_lane1_y1) / (left_lane1_x2 - left_lane1_x1)
            right_lane1_slope = (right_lane1_y2 - right_lane1_y1) / (right_lane1_x2 - right_lane1_x1)

            left_lane2_slope = (left_lane2_y2 - left_lane2_y1) / (left_lane2_x2 - left_lane2_x1)
            right_lane2_slope = (right_lane2_y2 - right_lan2_y1) / (right_lane2_x2 - right_lane2_x1)

            try:
                # Calculate the percent difference between the lane and road
                percent_diff_left = abs((left_lane1_slope - left_lane2_slope) / left_lane2_slope)
                percent_diff_right = abs((right_lane1_slope - right_lane2_slope) / right_lane2_slope)

                return [percent_diff_left, percent_diff_right]
            # One of the slopes could be zero or close to it,
            except ZeroDivisionError as e:
                return [100]
        else:
            return [100]

    # If there is only one lane
    elif lane_detection1 == 2 or lane_detection1 == 3:
        if lane_detection1 == lane_detection2:

            lane1_x1, lane1_y1, lane1_x2, lane1_y2 = lanes_1[0]
            lane1_slope = (lane1_y2 - lane1_y1) / (lane1_x2 - lane1_x1)

            lane2_x1, lane2_y1, lane2_x2, lane2_y2 = lanes_2[0]
            lane2_slope = (lane2_y2 - lane2_y1) / (lane2_x2 - lane2_x1)

            try:
                percent_diff = abs((lane1_slope - lane2_slope) / lane2_slope)
                return [percent_diff]
            except ZeroDivisionError as e:
                return [100]

        # No lanes detected
        else:
            return [100]
    else:
        return [100]


# temp, formatting is kinda wonky temp fix
def format_lanes(lanes):
    if lanes:
        if len(lanes) == 2:
            return [list(lanes[0]), list(lanes[1])]
        elif len(lanes) == 1:
            return [list(lanes[0])]
    else:
        return []
