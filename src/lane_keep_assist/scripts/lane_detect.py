#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import math
from typing import List, Tuple


HOUGH_MIN_VOTES = 20
HOUGH_MIN_LINE_LENGTH = 30
HOUGH_MAX_LINE_GAP = 5

CANNY_MIN_THRESHOLD = 40
CANNY_MAX_THRESHOLD = 80

# HLS thresholding on the image
def hls_thresholding(image: np.ndarray) -> np.ndarray:
    hls_image = cv.cvtColor(image, cv.COLOR_BGR2HLS)

    # Increase image saturation, increases chance of detecting yellow.
    (h, l, s) = cv.split(hls_image)
    s = s * 4
    s = np.clip(s, 0, 255)
    hls_image = cv.merge([h, l, s])

    # Yellow lower and upper threshold values
    yellow_lower = np.array([20, 120, 40], dtype="uint8")
    yellow_upper = np.array([45, 255, 255], dtype="uint8")

    # Upper and lower thresholds for a white or very close to white color
    lower_white = np.array([0, 230, 0], dtype="uint8")
    upper_white = np.array([255, 255, 255], dtype="uint8")

    # Apply the yellow thresholding
    yellow_mask = cv.inRange(hls_image, yellow_lower, yellow_upper)
    white_mask = cv.inRange(hls_image, lower_white, upper_white)

    yellow_white_mask = cv.bitwise_or(yellow_mask, white_mask)

    return yellow_white_mask


# Edge detection on the image
def gradient_thresholding(gray: np.ndarray) -> np.ndarray:
    # Min and max thresholds for the gradient magnitude to determine if it is an edge
    canny_detection = cv.Canny(gray, CANNY_MIN_THRESHOLD, CANNY_MAX_THRESHOLD)

    return canny_detection


# Try and mask out areas that are not of concern, for the scene image detection it attempts
# to cut out as many regions as possible.
def mask_image(edges: np.ndarray) -> np.ndarray:
    height, width = edges.shape
    # Define the polygonal region we want to look at.
    polygons = np.array([
        [(0, height - 150), (0, height), (width, height), (width, height - 150), (width / 2, 150)]
    ], np.int32)
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(edges)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(edges, mask)
    return segment


# Create a mask for the segmented image detection, only look at the lower half of the screen
def mask_segmented_image(edges: np.ndarray) -> np.ndarray:
    height, width = edges.shape
    # Define the polygonal region we want to look at.
    polygons = np.array([[
        (0, height * 1 / 2),
        (width, height * 1 / 2),
        (width, height),
        (0, height),
    ]], np.int32)
    # Creates an image filled with zero intensities with the same dimensions as the frame
    mask = np.zeros_like(edges)
    # Allows the mask to be filled with values of 1 and the other areas to be filled with values of 0
    cv.fillPoly(mask, polygons, 255)
    # A bitwise and operation between the mask and frame keeps only the triangular area of the frame
    segment = cv.bitwise_and(edges, mask)
    return segment


# Detect the hough lines
def hough_line_detection(edges: np.ndarray):
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = HOUGH_MIN_VOTES  # minimal of votes
    line_segments = cv.HoughLinesP(edges, rho, angle, min_threshold,
                                   np.array([]), minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)

    return line_segments


# Hough line interpolation
def make_points(frame: np.ndarray, line: np.ndarray) -> List[List[float]]:
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    try:
        x1 = max(-width, min(2 * width, int((y1 - intercept) / slope)))
        x2 = max(-width, min(2 * width, int((y2 - intercept) / slope)))
        return [[x1, y1, x2, y2]]
    except OverflowError as e:
        return [[0, 0, 0, 0]]


def average_slope_intercept(frame: np.ndarray, line_segments: np.ndarray):
    """
    This function combines line segments into one or two lane lines
    If all line slopes are < 0: then we only have detected left lane
    If all line slopes are > 0: then we only have detected right lane
    """
    lane_lines = []
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = 1 / 3
    left_region_boundary = width * (1 - boundary)  # left lane line segment should be on left 2/3 of the screen
    right_region_boundary = width * boundary  # right lane line segment should be on left 2/3 of the screen

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]
            if slope < 0:
                if x1 < left_region_boundary and x2 < left_region_boundary:
                    left_fit.append((slope, intercept))
            else:
                if x1 > right_region_boundary and x2 > right_region_boundary:
                    right_fit.append((slope, intercept))

    left_fit_average = np.average(left_fit, axis=0)
    if len(left_fit) > 0:
        lane_lines.append(make_points(frame, left_fit_average))

    right_fit_average = np.average(right_fit, axis=0)
    if len(right_fit) > 0:
        lane_lines.append(make_points(frame, right_fit_average))

    return lane_lines


# Detect the most common colour in the lower quarter of the screen where the road is assured to be
# This most common colour should be the road region.
# From this the road is isolated from the other detections.
def get_road_detection(image: np.ndarray) -> (np.ndarray, int):
    detections = []
    height, width, _ = image.shape

    for y in range(math.floor(height * 3 / 4), height):
        for x in range(math.floor(width * 3 / 4), width):
            detections.append(np.sum(image[y][x]))

    most_common_detection = max(set(detections), key=detections.count)

    new_image = np.zeros((height, width))

    for y in range(0, height):
        for x in range(0, width):
            sum = np.sum(image[y][x])
            if sum == most_common_detection:
                new_image[y][x] = 255

    return new_image.astype(np.uint8), most_common_detection


# Detect the lines of both the lanes and the road bounds
def lane_detect(cv_img: np.ndarray, segmented_image: np.ndarray):
    gray_image = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

    # Gradient and HLS thresholding
    gradient_threshold = gradient_thresholding(gray_image)
    hls_threshold = hls_thresholding(cv_img)

    # gradient threshold of segmentation
    segmented_road, road_colour = get_road_detection(segmented_image)
    #Bluring the image to try and reduce the affect of aliasing
    segmented_gray_blur = cv.blur(segmented_road, (4, 4))
    gradient_threshold_segmentation = gradient_thresholding(segmented_gray_blur)

    # Analyse the actual camera image

    # Combine the two thresholding methods
    combined = cv.bitwise_or(gradient_threshold, hls_threshold)
    # Mask the image
    masked_image = mask_image(combined)
    # Perform the hough transform
    hough_lines = hough_line_detection(masked_image)
    # Get the lanes from the hough lines
    lane_lines = average_slope_intercept(cv_img, hough_lines)

    # Analyse the segmentation image
    masked_segmented = mask_segmented_image(gradient_threshold_segmentation)
    # Perform the hough transform
    hough_lines_segmented = hough_line_detection(masked_segmented)
    # Get the lanes from the hough lines
    lane_lines_segmented = average_slope_intercept(cv_img, hough_lines_segmented)

    # Saved for live visualization
    # for line in hough_lines_segmented:
    #     for x1, y1, x2, y2 in line:
    #         cv.line(cv_img, (x1, y1), (x2, y2), (0, 0, 255), 2)

    # if lane_lines:
    #     coord1, coord2, coord3, coord4 = lane_lines[0][0]
    #     cv.line(cv_img, (coord1, coord2), (coord3, coord4), (0, 255, 0), 2)
    #
    #     coord1, coord2, coord3, coord4 = lane_lines[1][0]
    #     cv.line(cv_img, (coord1, coord2), (coord3, coord4), (0, 255, 0), 2)
    # cv.waitKey(1)

    return lane_lines, lane_lines_segmented, road_colour
