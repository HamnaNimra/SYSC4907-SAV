#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import time
import math
import lane_detect
from typing import List, Tuple
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from lane_bound_status import LaneBoundStatus


# Returns the results of both detection methods lane and approximate road boundaries
# Returns the percent difference between the two methods detection if they agree on what type of detection
# Returns the coordinates of the two lines
# Returns the current majority segment found in the segmentation image by the sum of the RGB value
def lane_interpret(img: np.ndarray, segmented_img: np.ndarray) -> Tuple[Tuple[LaneBoundStatus, LaneBoundStatus, LaneBoundStatus],
                                                                        Tuple[float, float], Tuple[List[float], List[float], List[float]], int]:
    bridge = CvBridge()
    height = img.height

    if img and segmented_img:
        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
            cv_segmented_img = bridge.imgmsg_to_cv2(segmented_img, desired_encoding="passthrough")

            # Get a result from both the normal camera image and the segmentation image
            gradient_lanes, hls_lanes, segmentation_lanes, road_colour = lane_detect.lane_detect(cv_img, cv_segmented_img)

            # Get the lane results
            gradient_result = lane_type(gradient_lanes)
            hls_result = lane_type(hls_lanes)
            segmentation_result = lane_type(segmentation_lanes)

            # Get the percent differences for the detections
            gradient_difference = compare_lanes(gradient_lanes, segmentation_lanes, gradient_result,
                                                segmentation_result)
            hls_difference = compare_lanes(hls_lanes, segmentation_lanes, hls_result, segmentation_result)

            return ((gradient_result, hls_result, segmentation_result), (gradient_difference, hls_difference),
                    (gradient_lanes, hls_lanes, segmentation_lanes), road_colour)

        except CvBridgeError as e:
            print(e)
            return (LaneBoundStatus.NO_BOUNDS, LaneBoundStatus.NO_BOUNDS, LaneBoundStatus.NO_BOUNDS), (100, 100), ([], [], []), 1


# Returns the type of lane detection
# 1 is two bounds
# 2 is one bound off to the left
# 3 is one bound off to the right
# 4 is no bounds
def lane_type(lines) -> LaneBoundStatus:
    if lines:
        if len(lines) == 2:
            return LaneBoundStatus.TWO_BOUNDS
        elif len(lines) == 1:
            x1, y1, x2, y2 = lines[0]
            slope = (y2-y1) / (x2-x1)

            if slope < 0:
                return LaneBoundStatus.ONE_BOUND_LEFT
            if slope > 0:
                return LaneBoundStatus.ONE_BOUND_RIGHT
    else:
        return LaneBoundStatus.NO_BOUNDS


# Get the percentage difference between the two lane detections
# lane_bound_detection and lane_bound_ground_truth are compared, a percent difference is given
# lane_bound_status and lane_bound_ground_truth_status both are the the result of the lane_type method
def compare_lanes(lane_bound_detection, lane_bound_ground_truth, lane_bound_type, lane_bound_ground_truth_type) -> List[float]:
    # If there is two lanes
    if lane_bound_type == LaneBoundStatus.TWO_BOUNDS:
        # If the lanes are the same get the percent difference
        if lane_bound_type == lane_bound_ground_truth_type:

            # Get the slopes of each lane detection
            left_lane1_x1, left_lane1_y1, left_lane1_x2, left_lane1_y2 = lane_bound_detection[0]
            right_lane1_x1, right_lane1_y1, right_lane1_x2, right_lane1_y2 = lane_bound_detection[1]

            left_lane2_x1, left_lane2_y1, left_lane2_x2, left_lane2_y2 = lane_bound_ground_truth[0]
            right_lane2_x1, right_lan2_y1, right_lane2_x2, right_lane2_y2 = lane_bound_ground_truth[1]

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
    elif lane_bound_type == LaneBoundStatus.ONE_BOUND_LEFT or lane_bound_type == LaneBoundStatus.ONE_BOUND_RIGHT:
        if lane_bound_type == lane_bound_ground_truth_type:

            lane1_x1, lane1_y1, lane1_x2, lane1_y2 = lane_bound_detection[0]
            lane1_slope = (lane1_y2 - lane1_y1) / (lane1_x2 - lane1_x1)

            lane2_x1, lane2_y1, lane2_x2, lane2_y2 = lane_bound_ground_truth[0]
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
