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
    iou_values_hough_gradient = []
    iou_values_hough_hls = []
    iou_values_hough_segmented = []
    index = 1
    for (original_image, segmented_image, annotated_image) in images:

        iou_hough = get_iou_of_hough_lines(original_image, segmented_image, annotated_image)
        iou_values_hough_gradient.append((index, iou_hough[0]))
        iou_values_hough_hls.append((index, iou_hough[1]))
        iou_values_hough_segmented.append((index, iou_hough[2]))
        index = index + 1

    with open("metrics.txt", "w") as file:
        file.write(f"Values after hough with gradient {iou_values_hough_gradient}\n")
        file.write(f"Values after hough with hls {iou_values_hough_hls}\n")
        file.write(f"Values after hough with segmented {iou_values_hough_segmented}\n")


# Gets the images defined in the json file from the test images file
def get_images() -> List[List[np.ndarray]]:
    file_path = os.path.abspath(os.path.dirname(__file__))
    path, _ = os.path.split(file_path)
    image_folder_path = os.path.join(path, "test_images")
    original_image_folder = os.path.join(image_folder_path, "original")
    annotated_image_folder = os.path.join(image_folder_path, "annotated")
    segmented_image_folder = os.path.join(image_folder_path, "segmented")

    images = {}
    for file in os.listdir(original_image_folder):
        result = re.search('[o]+[_]+(\d*)', file)
        number = int(result.group(1))
        images[number] = [cv.imread(os.path.join(original_image_folder, file))]

    for file in os.listdir(segmented_image_folder):
        result = re.search('[s]+[_]+(\d*)', file)
        number = int(result.group(1))
        images[number].append(cv.imread(os.path.join(segmented_image_folder, file)))

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


# Gets the iou before after the hough transform
def get_iou_of_hough_lines(original_image: np.ndarray, segmented_image: np.ndarray, annotated_image: np.ndarray) -> Tuple[float, float, float]:

    gradient_lane_lines, hls_lane_lines, segmented_lane_lines, _ = lane_detect(original_image, segmented_image)

    annotation = get_annotation(annotated_image)

    gradient_lane_drawing = np.zeros((360, 640, 3), np.uint8)
    hls_lane_drawing = np.zeros((360, 640, 3), np.uint8)
    segmented_lane_drawing = np.zeros((360, 640, 3), np.uint8)

    # Put the lane lines into a black image
    gradient_lane_drawing = draw_lines_image(gradient_lane_drawing, gradient_lane_lines)
    hls_lane_drawing = draw_lines_image(hls_lane_drawing, hls_lane_lines)
    segmented_lane_drawing = draw_lines_image(segmented_lane_drawing, segmented_lane_lines)

    # Flatten to gray scale
    gradient_lane_drawing = cv.cvtColor(gradient_lane_drawing, cv.COLOR_BGR2GRAY)
    hls_lane_drawing = cv.cvtColor(hls_lane_drawing, cv.COLOR_BGR2GRAY)
    segmented_lane_drawing = cv.cvtColor(segmented_lane_drawing, cv.COLOR_BGR2GRAY)

    # Not the proper way to get IoU, but this should give a more accurate look at the affect of noise
    gradient_results = calc_faux_iou(gradient_lane_drawing, annotation)
    hls_results = calc_faux_iou(hls_lane_drawing, annotation)
    segmented_results = calc_faux_iou(segmented_lane_drawing, annotation)

    return gradient_results, hls_results, segmented_results


# Draw lines on the image
def draw_lines_image(image: np.ndarray, lines):
    if lines:
        for line in lines:
            coord1, coord2, coord3, coord4 = line
            cv.line(image, (coord1, coord2), (coord3, coord4), (255, 255, 255), 2)
    return image


# Not the correct way to calculate IOU, but gets the message across
def calc_faux_iou(lane_drawing, annotation):
    total_detection = np.sum(lane_drawing == 255)
    correct_detection = np.sum(cv.bitwise_and(lane_drawing, annotation) == 255)
    if total_detection == 0:
        return 0
    else:
        return correct_detection / total_detection * 100