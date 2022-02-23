#!/usr/bin/env python3
import pandas as pd
import torch
import rospy
# TODO: create DetectionResult msg
from sign_car_recognition.msg import DetectionResult, DetectionResults
# ROS Image message
from sensor_msgs.msg import Image, CompressedImage


# Stubbed class
class SignDetector:
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        self.model.eval()
        # New topic
        self.pub = rospy.Publisher('sign_detection', DetectionResults, queue_size=10)

    def listen(self):
        rospy.init_node('sign_detector', anonymous=True)
        # This is existing topic from prev years
        rospy.Subscriber('airsim/image_uint8_png', CompressedImage, self.handle_image)
        rospy.spin()

    # Do detection on an image and publish the result
    def handle_image(self, cmp_img: CompressedImage):
        f = open('./tmp.png', 'wb')
        f.write(cmp_img.data)
        f.close()

        res = self.detect_signs()
        self.pub.publish(res)

    # Detect signs given an image
    def detect_signs(self):
        # return detection results consisting of bounding boxes and classes
        results = self.model('tmp.png')
        results.print()
        res_list = results.pandas().xyxy[0].to_numpy().tolist()
        rospy.loginfo(res_list)

        detect_results = []
        for elem in res_list:
            dr = DetectionResult()
            dr.xmin = elem[0]
            dr.ymin = elem[1]
            dr.xmax = elem[2]
            dr.ymax = elem[3]
            dr.confidence = elem[4]
            dr.class_num = elem[5]
            dr.name = elem[6]
            detect_results.append(dr)

        # Return no detections
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
    sd.listen()
