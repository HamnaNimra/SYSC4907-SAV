#!/usr/bin/env python3
import pandas as pd
import torch
import rospy
import airsim
import numpy as np
import cv2
from sign_car_recognition.msg import DetectionResult, DetectionResults
from sensors.msg import SceneDepth
# ROS Image message
from sensor_msgs.msg import Image
from std_msgs.msg import String
from typing import List
import math


# Stubbed class
class SignDetector:
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.model.eval()
        # New topic
        self.pub = rospy.Publisher('object_detection', DetectionResults, queue_size=1)
        rospy.init_node('sign_detector', anonymous=True)

    def listen(self):
        # This is existing topic from prev years
        rospy.Subscriber('airsim/scene_depth', SceneDepth, self.handle_image)
        rospy.spin()

    # Do detection on an image and publish the detections array
    def handle_image(self, combine: SceneDepth):
        img1d = np.frombuffer(combine.scene.data, dtype=np.uint8)
        # reshape array to 3 channel image array
        img_rgb = img1d.reshape(combine.scene.height, combine.scene.width, 3)

        # f = open(f'/home/mango/test_imgs/n_{rospy.Time.now()}.txt', 'wb')
        # f.write(img.data)
        # f.close()
        # cv2.imwrite(f'/home/mango/test_imgs/n_{rospy.Time.now()}.png', img_rgb)

        # Run object detection on scene data
        res: List[DetectionResult] = self.detect_objects(img_rgb)

        # Match with scene depth
        depth = np.array(combine.depth_data, dtype=np.float32)
        depth = depth.reshape(combine.scene.height, combine.scene.width)
        depth = np.array(depth * 255, dtype=np.uint8)

        # Find median depth value for each detection box
        for detect in res:
            x1, x2, y1, y2 = math.floor(detect.xmin), math.floor(detect.xmax), math.floor(detect.ymin), math.floor(detect.ymax)
            xdif, ydif = x2-x1, y2-y1
            x1_s, x2_s, y1_s, y2_s = math.floor(x1+0.20*xdif), math.floor(x2-0.20*xdif), math.floor(y1+0.20*ydif), math.floor(y2-0.20*ydif)

            sub = depth[y1_s:y2_s, x1_s:x2_s]
            med = np.median(sub)

            if math.isnan(med):
                rospy.loginfo(sub)
                rospy.loginfo(sub.shape)

            # Range of values is 0 to 100 m
            detect.depth = med / 2.56  # (256/100)

            # Draw bounding boxes
            # cv2.rectangle(depth, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # cv2.putText(depth, f'{detect.name}: {detect.depth}', (x2+10, y2), 0, 0.3, (0, 255, 0))

            cv2.rectangle(img_rgb, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(img_rgb, f'{detect.name}: {detect.depth}', (x2+10, y2), 0, 0.3, (0, 255, 0))

        # Write debug images to visualize the detections.
        # DONT LEAVE THIS ON FOR LONG PERIODS OF TIME OR YOU WILL FILL YOUR HARD DRIVE WITH PNGS
        # cv2.imwrite(f'/home/mango/test_imgs/n_{rospy.Time.now()}_d.png', depth)
        # cv2.imwrite(f'/home/mango/test_imgs/n_{rospy.Time.now()}_s.png', img_rgb)
        cv2.imshow("object_detections", img_rgb)
        cv2.waitKey(1)

        rospy.loginfo(res)
        self.pub.publish(res)

    # Detect objects given an image (np array)
    def detect_objects(self, img):
        # return detection results consisting of bounding boxes and classes
        results = self.model(img)
        results.print()
        res_list = results.pandas().xyxy[0].to_numpy().tolist()

        detect_results = []
        # important detection classes we care about
        imp_classes = {
            0: 'person',
            2: 'car',
            3: 'motorcycle',
            7: 'truck',
            9: 'traffic light',
            11: 'stop sign'
        }
        for elem in res_list:
            # Skip adding the result if not a relevant class
            if elem[5] not in imp_classes:
                continue

            dr = DetectionResult()
            dr.xmin = elem[0]
            dr.ymin = elem[1]
            dr.xmax = elem[2]
            dr.ymax = elem[3]
            dr.confidence = elem[4]
            dr.class_num = elem[5]
            dr.name = elem[6]
            detect_results.append(dr)

        return detect_results

    # For testing purposes when you don't want to run the whole ROS thing
    def test_detect(self, file_path):
        # return detection results consisting of bounding boxes and classes
        results = self.model(file_path)
        results.print()
        pd_res = results.pandas()
        rospy.loginfo(pd_res.xyxy[0])


# Give the option to run separately
if __name__ == "__main__":
    sd = SignDetector()
    # sd.test_detect('../test_imgs/Screenshot 2022-01-18 175150.jpg')

    ready_pub = rospy.Publisher("ready", String, queue_size=1)
    ready_pub.publish('SignDetect')

    sd.listen()
