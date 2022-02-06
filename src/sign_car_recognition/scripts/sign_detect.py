#!/usr/bin/env python3
import torch
import rospy
# TODO: create DetectionResult msg
from sign_car_recognition.msg import DetectionResult
# ROS Image message
from sensor_msgs.msg import Image, CompressedImage


# Stubbed class
class SignDetector:
    def __init__(self):
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
        # self.fake_model = []
        self.model.eval()
        # New topic
        self.pub = rospy.Publisher('sign_detection', DetectionResult, queue_size=10)

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
        # self.pub.publish(res)

    # Detect signs given an image
    def detect_signs(self):
        # return detection results consisting of bounding boxes and classes
        results = self.model('tmp.png')
        results.print()
        rospy.loginfo(results.pandas().xyxy[0])

        # Return no detections
        return DetectionResult()


# Give the option to run separately
if __name__ == "__main__":
    sd = SignDetector()
    sd.listen()
