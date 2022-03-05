#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import math
from typing import List, Tuple

HOUGH_MIN_VOTES = 50
HOUGH_MIN_LINE_LENGTH = 60
HOUGH_MAX_LINE_GAP = 10

HOUGH_MIN_VOTES_SEGMENTED = 10
HOUGH_MIN_LINE_LENGTH_SEGMENTED = 50
HOUGH_MAX_LINE_GAP_SEGMENTED = 5

CANNY_MIN_THRESHOLD = 150
CANNY_MAX_THRESHOLD = 200


# HLS thresholding on the image
def hls_thresholding(image: np.ndarray) -> np.ndarray:
    hls_image = cv.cvtColor(image, cv.COLOR_BGR2HLS)

    # Increase image saturation, increases chance of detecting yellow.
    (h, l, s) = cv.split(hls_image)
    s = s * 4
    s = np.clip(s, 0, 255)
    hls_image = cv.merge([h, l, s])

    # Yellow lower and upper threshold values
    yellow_lower = np.array([10, 120, 40], dtype="uint8")
    yellow_upper = np.array([65, 255, 255], dtype="uint8")

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
    # Cut off the top half
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
def hough_line_detection(edges: np.ndarray) -> List[Tuple[float, float, float, float]]:
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = HOUGH_MIN_VOTES  # minimal of votes
    line_segments = cv.HoughLinesP(edges, rho, angle, min_threshold,
                                   np.array([]), minLineLength=HOUGH_MIN_LINE_LENGTH, maxLineGap=HOUGH_MAX_LINE_GAP)

    return line_segments


# Detect the hough lines, different methods to try using different parameters for segmented image
def hough_line_detection_segmentation(edges: np.ndarray) -> List[Tuple[float, float, float, float]]:
    rho = 1  # distance precision in pixel, i.e. 1 pixel
    angle = np.pi / 180  # angular precision in radian, i.e. 1 degree
    min_threshold = HOUGH_MIN_VOTES_SEGMENTED  # minimal of votes
    line_segments = cv.HoughLinesP(edges, rho, angle, min_threshold,
                                   np.array([]), minLineLength=HOUGH_MIN_LINE_LENGTH_SEGMENTED, maxLineGap=HOUGH_MAX_LINE_GAP_SEGMENTED)

    return line_segments


# Get two points from a line
def make_points(frame: np.ndarray, line: np.ndarray) -> List[List[float]]:
    height, width, _ = frame.shape
    slope, intercept = line
    y1 = height  # bottom of the frame
    y2 = int(y1 * 1 / 2)  # make points from middle of the frame down

    # bound the coordinates within the frame
    try:
        x1 = int((y1 - intercept) / slope)
        if not 0 <= x1 <= width:
            if slope < 0:
                x1 = 0
            else:
                x1 = width
            y1 = int(x1*slope + intercept)
        x2 = int((y2 - intercept) / slope)
        if not 0 <= x2 <= width:
            if slope > 0:
                x2 = 0
            else:
                x2 = width
            y2 = int(x2*slope + intercept)
        return [x1, y1, x2, y2]
    except OverflowError as e:
        return [0, 0, 0, 0]


# Divide the lanes into left and right segments
def average_slope_intercept(frame: np.ndarray, line_segments: np.ndarray) -> List[float]:
    lane_lines = []
    if line_segments is None:
        return lane_lines

    height, width, _ = frame.shape
    left_fit = []
    right_fit = []

    boundary = width * 1 / 2

    for line_segment in line_segments:
        for x1, y1, x2, y2 in line_segment:
            if x1 == x2:
                continue
            fit = np.polyfit((x1, x2), (y1, y2), 1)
            slope = fit[0]
            intercept = fit[1]

            # Try and remove more horizontal hough lines that are likely not lanes
            if 0.1 > slope > -0.1:
                continue

            if x1 < boundary and x2 < boundary:
                left_fit.append((slope, intercept))
            elif x1 > boundary and x2 > boundary:
                right_fit.append((slope, intercept))
            else:
                continue

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
def lane_detect(cv_img: np.ndarray, segmented_image: np.ndarray) -> Tuple[List, List, List, int]:
    gray_image = cv.cvtColor(cv_img, cv.COLOR_BGR2GRAY)

    # Gradient threshold of scene image
    gradient_threshold = gradient_thresholding(gray_image)

    # HLS threshold of scene image
    hls_threshold = hls_thresholding(cv_img)

    # Getting the road/main detection of the segmented image
    segmented_road, road_colour = get_road_detection(segmented_image)
    # Blurring the image to try and reduce the affect of aliasing
    segmented_gray_blur = cv.blur(segmented_road, (4, 4))
    gradient_threshold_segmentation = gradient_thresholding(segmented_gray_blur)
    # Doing the same thing, but this time to possible increase the size of the lines and again reduce aliasing
    gradient_threshold_segmentation = cv.blur(gradient_threshold_segmentation, (3, 3))
    gradient_threshold_segmentation[gradient_threshold_segmentation > 0] = 255

    # Apply half mask to both images
    masked_segmented = mask_image(gradient_threshold_segmentation)
    masked_gradient = mask_image(gradient_threshold)
    masked_hls = mask_image(hls_threshold)

    # Only look at objects that are on the ground for gradient and hls
    masked_gradient = cv.bitwise_and(masked_gradient, segmented_road)
    masked_hls = cv.bitwise_and(masked_hls, segmented_road)

    # Perform the hough transform on each detection
    hough_gradient = hough_line_detection(masked_gradient)
    hough_segmented = hough_line_detection_segmentation(masked_segmented)
    hough_hls = hough_line_detection(masked_hls)

    # Get the lanes from each detection
    gradient_lane_lines = average_slope_intercept(cv_img, hough_gradient)
    segmented_lane_lines = average_slope_intercept(cv_img, hough_segmented)
    hls_lane_lines = average_slope_intercept(cv_img, hough_hls)

    # # Saved for live visualization assumes two lanes
    # try:
    #     masked_segmented = cv.cvtColor(masked_segmented, cv.COLOR_GRAY2RGB)
    #     if hough_segmented is not None:
    #         for line in hough_segmented:
    #             for x1, y1, x2, y2 in line:
    #                 fit = np.polyfit((x1, x2), (y1, y2), 1)
    #                 slope = fit[0]
    #                 if 0.1 > slope > -0.1:
    #                     continue
    #                 cv.line(masked_segmented, (x1, y1), (x2, y2), (255, 0, 0), 2)
    #
    #     if gradient_lane_lines:
    #         for line in gradient_lane_lines:
    #             coord1, coord2, coord3, coord4 = line
    #             cv.line(masked_segmented, (coord1, coord2), (coord3, coord4), (0, 0, 255), 2)
    #
    #     if segmented_lane_lines:
    #         for line in segmented_lane_lines:
    #             coord1, coord2, coord3, coord4 = line
    #             cv.line(masked_segmented, (coord1, coord2), (coord3, coord4), (0, 255, 0), 2)
    #     cv.imshow("masked", masked_segmented)
    #     cv.imshow("demo", cv_img)
    #     cv.waitKey(1)
    # except IndexError as e:
    #     print(e)

    return gradient_lane_lines, hls_lane_lines, segmented_lane_lines, road_colour
