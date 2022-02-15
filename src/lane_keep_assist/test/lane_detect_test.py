import cv2 as cv
import numpy as np
from scripts.lane_detect import *
import os
import json
import re
from typing import List, Tuple


# Returns metrics on the iou of the images
def measure_detect():
    images = get_images()
    iou_values_before_hough = []
    iou_values_hough = []
    index = 1
    for (original_image, annotated_image) in images:
        iou_before_hough = get_iou_of_scan(original_image, annotated_image)
        iou_values_before_hough.append((index, iou_before_hough))

        iou_hough = get_iou_of_hough_lines(original_image, annotated_image)
        iou_values_hough.append((index, iou_hough))
        index = index + 1

    with open("metrics.txt", "w") as file:
        file.write(f"Values before hough {iou_values_before_hough}\n")
        file.write(f"Values after hough {iou_values_hough}\n")


# Gets the images defined in the json file from the test images file
def get_images() -> List[List[np.ndarray]]:
    file_path = os.path.abspath(os.path.dirname(__file__))
    path, _ = os.path.split(file_path)
    image_folder_path = os.path.join(path, "test_images")
    original_image_folder = os.path.join(image_folder_path, "original")
    annotated_image_folder = os.path.join(image_folder_path, "annotated")

    images = {}
    for file in os.listdir(original_image_folder):
        result = re.search('[o]+[_]+(\d*)', file)
        number = int(result.group(1))
        images[number] = [cv.imread(os.path.join(original_image_folder, file))]

    for file in os.listdir(annotated_image_folder):
        result = re.search('[a]+[_]+(\d*)', file)
        number = int(result.group(1))
        images[number].append(cv.imread(os.path.join(annotated_image_folder, file)))
    return list(images.values())


# Gets just the annotation from the image
def get_annotation(annotated_image: np.ndarray) -> np.ndarray:
    # Get just the annotation from the annotated image
    # Need to get two masks as red wraps around the colour wheel
    hls_annotation = cv.cvtColor(annotated_image, cv.COLOR_BGR2HLS)
    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask1 = cv.inRange(hls_annotation, lower_red, upper_red)
    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask2 = cv.inRange(hls_annotation, lower_red, upper_red)
    annotation = cv.bitwise_or(mask1, mask2)
    return annotation


# Gets the iou before before the hough transform, a better indication on how well the lanes are being found
def get_iou_of_scan(image: np.ndarray, annotated_image: np.ndarray) -> float:
    # Lane detection with original image
    gradient_threshold = gradient_thresholding(image)
    hls_threshold = hls_thresholding(image)
    combined_detection = cv.bitwise_or(gradient_threshold, hls_threshold)
    combined_detection_with_mask = mask_image(combined_detection)

    annotation = get_annotation(annotated_image)

    # Not the proper way to get IoU, but this should give a more accurate look at the affect of noise
    total_detection_overlap = np.sum(cv.bitwise_or(combined_detection_with_mask, annotation) == 255)
    correct_detection = np.sum(cv.bitwise_and(combined_detection_with_mask, annotation) == 255)

    return correct_detection / total_detection_overlap * 100


# Gets the iou before after the hough transform
def get_iou_of_hough_lines(image: np.ndarray, annotated_image: np.ndarray) -> float:
    # Lane detection with original image
    gradient_threshold = gradient_thresholding(image)
    hls_threshold = hls_thresholding(image)
    combined_detection = cv.bitwise_or(gradient_threshold, hls_threshold)
    combined_detection_with_mask = mask_image(combined_detection)
    hough_lines = hough_line_detection(combined_detection_with_mask)
    lane_lines = average_slope_intercept(image, hough_lines)

    annotation = get_annotation(annotated_image)

    lane_drawing = np.zeros((360, 640, 3), np.uint8)

    # Put the lane lines into a black image
    if lane_lines:
        for lane in lane_lines:
            coord1, coord2, coord3, coord4 = lane[0]
            cv.line(lane_drawing, (coord1, coord2), (coord3, coord4), (255, 255, 255), 2)
    else:
        return 0

    # Flatten to gray scale
    lane_drawing = cv.cvtColor(lane_drawing, cv.COLOR_BGR2GRAY)

    # Not the proper way to get IoU, but this should give a more accurate look at the affect of noise
    total_detection_overlap = np.sum(cv.bitwise_or(lane_drawing, annotation) == 255)
    correct_detection = np.sum(cv.bitwise_and(lane_drawing, annotation) == 255)

    return correct_detection / total_detection_overlap * 100
