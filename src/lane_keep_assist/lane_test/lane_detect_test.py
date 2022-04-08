from scripts.lane_detect import *
from scripts.lane_bound_status import LaneBoundStatus
from sklearn import metrics
from typing import List, Tuple
from scripts.lane_detect import *
import numpy as np
import cv2 as cv
import os
import re

# Constants
TL = 'two_lanes'
OLL = 'one_lane_left'
OLR = 'one_lane_right'
NL = 'no lanes'
CLASSIFICATION_MAP = {LaneBoundStatus.TWO_BOUNDS: TL, LaneBoundStatus.ONE_BOUND_LEFT: OLL,
                      LaneBoundStatus.ONE_BOUND_RIGHT: OLR, LaneBoundStatus.NO_BOUNDS: NL}
CLASSIFICATION_MAP_ugh = {'TL': TL, 'OLL': OLL, 'OLR': OLR, 'NL': NL}


# Returns metrics on the iou of the images
def measure_detect():
    images, correct_classifications = get_images()
    iou_values_hough_gradient = []
    iou_values_hough_hls = []
    iou_values_hough_segmented = []
    index = 1
    classifications_gradient = []
    classifications_hls = []
    classifications_segmented = []

    for (original_image, segmented_image, annotated_image) in images:
        iou_hough, classification = get_iou(original_image, segmented_image, annotated_image)
        classifications_gradient.append(CLASSIFICATION_MAP[classification[0]])
        classifications_hls.append(CLASSIFICATION_MAP[classification[1]])
        classifications_segmented.append(CLASSIFICATION_MAP[classification[2]])
        iou_values_hough_gradient.append((index, iou_hough[0]))
        iou_values_hough_hls.append((index, iou_hough[1]))
        iou_values_hough_segmented.append((index, iou_hough[2]))
        index = index + 1

    with open("metrics.txt", "w") as file:
        file.write(f"Values after hough with gradient {iou_values_hough_gradient}\n")
        file.write(f"Values after hough with hls {iou_values_hough_hls}\n")
        file.write(f"Values after hough with segmented {iou_values_hough_segmented}\n")

        file.write(f"average accuracy of gradient is {np.average([i[1] for i in iou_values_hough_gradient])}\n")
        file.write(f"average accuracy of hls is {np.average([i[1] for i in iou_values_hough_hls])}\n")
        file.write(f"average accuracy of segmented is {np.average([i[1] for i in iou_values_hough_segmented])}\n")
        file.write(metrics.classification_report(correct_classifications, classifications_gradient, digits=3))


# Gets the images defined in the json file from the test images file
def get_images() -> Tuple[List[List[np.ndarray]], List[str]]:
    file_path = os.path.abspath(os.path.dirname(__file__))
    path, _ = os.path.split(file_path)
    image_folder_path = os.path.join(path, "test_images")
    original_image_folder = os.path.join(image_folder_path, "original")
    annotated_image_folder = os.path.join(image_folder_path, "annotated")
    segmented_image_folder = os.path.join(image_folder_path, "segmented")
    classifications = []

    images = {}
    for file in os.listdir(original_image_folder):
        result = re.search('[o][_](\d*)', file)
        number = int(result.group(1))
        images[number] = [cv.imread(os.path.join(original_image_folder, file))]

    for file in os.listdir(segmented_image_folder):
        result = re.search('[s][_](\d*)', file)
        number = int(result.group(1))
        images[number].append(cv.imread(os.path.join(segmented_image_folder, file)))

    for file in os.listdir(annotated_image_folder):
        result = re.search('[a][_](\d*)[_](\w*)', file)
        number = int(result.group(1))
        classifications.append(CLASSIFICATION_MAP_ugh[result.group(2)])
        images[number].append(cv.imread(os.path.join(annotated_image_folder, file)))
    return list(images.values()), classifications


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


# Gets the IOU of the lines
def get_iou(original_image: np.ndarray, segmented_image: np.ndarray, annotated_image: np.ndarray) -> Tuple[Tuple[
                                                                                                               float, float, float],
                                                                                                           Tuple[
                                                                                                               LaneBoundStatus, LaneBoundStatus, LaneBoundStatus]]:
    # Detect the lanes
    gradient_lane_lines, hls_lane_lines, segmented_lane_lines, _ = lane_detect(original_image, segmented_image)

    # Get the classification
    gradient_class = lane_type(gradient_lane_lines)
    hls_class = lane_type(hls_lane_lines)
    segmentation_class = lane_type(segmented_lane_lines)

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

    return (gradient_results, hls_results, segmented_results), (gradient_class, hls_class, segmentation_class)


####################################################################
# The two methods below needed to be copied in due to pathing issues
####################################################################

# Draw lines on the image
def draw_lines_image(image: np.ndarray, lines):
    if lines:
        for line in lines:
            coord1, coord2, coord3, coord4 = line
            cv.line(image, (coord1, coord2), (coord3, coord4), (255, 255, 255), 2)
    return image


# Not the correct way to calculate IOU, but gets the message across
def calc_faux_iou(lane_drawing, annotation) -> float:
    total_detection = np.sum(lane_drawing == 255)
    correct_detection = np.sum(cv.bitwise_and(lane_drawing, annotation) == 255)
    if total_detection == 0:
        return 0
    else:
        return correct_detection / total_detection * 100


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
            slope = (y2 - y1) / (x2 - x1)

            if slope < 0:
                return LaneBoundStatus.ONE_BOUND_LEFT
            if slope > 0:
                return LaneBoundStatus.ONE_BOUND_RIGHT
    else:
        return LaneBoundStatus.NO_BOUNDS


def main():
    measure_detect()


if __name__ == "__main__":
    main()
