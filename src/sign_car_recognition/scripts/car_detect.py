#!/usr/bin/env python3
import torch
import rospy
import os
from sensor_msgs.msg import Image


# Note: This class is not used for now, the car and sign detection is done by the same node for now...
# The commented out code is kept here for future reference. If we need to train our own custom model in the future
# we will have to load it like this.
class YoloDetector:
    def __init__(self):
        # Do the setup first

        # Custom model
        # model = torch.hub.load('ultralytics/yolov5', 'custom', path='../best_yolo5_car.pt')

        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s')

        # checkpoint = torch.load('../best_yolo5_car.pt')
        # model.load_state_dict(checkpoint)
        # model.load_state_dict(checkpoint['best_fitness'])
        # optimizer.load_state_dict(checkpoint['optimizer'])
        self.model.eval()

    #
    def detect(self):
        # Do inference
        #
        # images = list(map(lambda x: '../test_imgs/' + x, os.listdir('../test_imgs/')))

        # TODO: Get images from Airsim

        # results = self.model(images)
        # results.print()
        # results.save()
        # print(results.pandas().xyxy[0])
        pass


if __name__ == '__main__':
    try:
        yd = YoloDetector()
        yd.detect()
    except rospy.ROSInterruptException as err:
        print(err)
        pass
