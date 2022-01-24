#!/usr/bin/env python3

import rospy
# TODO: create DetectionResult msg
from sign_car_recognition.msg import DetectionResult
# ROS Image message
from sensor_msgs.msg import Image


# Stubbed class
class SignDetector:
    def __init__(self):
        self.fake_model = []
        # New topic
        self.pub = rospy.Publisher('sign_detection', DetectionResult, queue_size=10)

    def listen(self):
        rospy.init_node('sign_detector', anonymous=True)
        # This is existing topic from prev years
        rospy.Subscriber('airsim/image_raw', Image, self.handle_image)
        rospy.spin()

    # Do detection on an image and publish the result
    def handle_image(self, image):
        res = self.detect_signs(image)
        self.pub.publish(res)

    # Detect signs given an image
    def detect_signs(self, image):
        # return detection results consisting of bounding boxes and classes

        # TODO: implement

        # Return no detections
        return DetectionResult()


# Give the option to run separately
if __name__ == "__main__":
    sd = SignDetector()
    sd.listen()
