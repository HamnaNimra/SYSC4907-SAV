#!/usr/bin/env python3

import airsim
import rospy
import cv2
import numpy as np

import os
from typing import List

# ROS Image message
from sensor_msgs.msg import Image, CompressedImage
from sensors.msg import SceneDepth


# Save the depth image for viewing purposes
def save_depth_img(response):
    depth = np.array(response.image_data_float, dtype=np.float32)
    depth = depth.reshape(response.height, response.width)
    depth = np.array(depth * 255, dtype=np.uint8)
    cv2.imwrite(f'/home/mango/depth_imgs/{rospy.Time.now()}_d.png', depth)


def airpub():
    pub = rospy.Publisher("airsim/image_raw", Image, queue_size=1)
    # Publish compressed images to the airsim topic
    pub2 = rospy.Publisher("airsim/image_uint8_png",  CompressedImage, queue_size=1)
    # Publish one message containing a scene image and a corresponding depth floats
    pub3 = rospy.Publisher("airsim/scene_depth",  SceneDepth, queue_size=1)

    rospy.init_node('image_raw', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # connect to the AirSim simulator 
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()

    # saved = False

    while not rospy.is_shutdown():
        # get camera images from the car
        responses: List[airsim.ImageResponse] = client.simGetImages([
            # png format
            airsim.ImageRequest(0, airsim.ImageType.Scene),
            # scene vision image in uncompressed RGB array
            airsim.ImageRequest(0, airsim.ImageType.Scene, False, False),
            # Depth image, these are values between 0 and 1 that map to AirSim distances of (0-100)m
            airsim.ImageRequest(0, airsim.ImageType.DepthVis, pixels_as_float=True)
        ])

        img_rgb_string = responses[1].image_data_uint8
        # rospy.loginfo(responses[0])

        # Uncompressed Image message of scene
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frameId"
        msg.encoding = "rgb8"
        msg.height = responses[1].height  # 360  # resolution should match values in settings.json
        msg.width = responses[1].width  # 640
        msg.data = img_rgb_string
        msg.is_bigendian = 0
        msg.step = msg.width * 3

        # Compressed image of scene
        # cmp_img = CompressedImage()
        # cmp_img.header.stamp = rospy.Time.now()
        # cmp_img.format = 'png'
        # cmp_img.data = responses[0].image_data_uint8

        # depth = np.array(responses[2].image_data_float, dtype=np.float32)
        # depth = depth.reshape(responses[2].height, responses[2].width)
        # depth = np.array(depth * 255, dtype=np.uint8)

        # Depth image
        # depth_img = Image()
        # depth_img.header.stamp = rospy.Time.now()
        # depth_img.format = 'png'
        # depth_img.data = depth.tolist()
        # depth_img.height = 360  # resolution should match values in settings.json
        # depth_img.width = 640

        # Depth + scene images
        combine = SceneDepth()
        combine.depth_data = responses[2].image_data_float
        combine.scene = msg

        # Publish the messages
        pub.publish(msg)
        # pub2.publish(cmp_img)
        pub3.publish(combine)

        # Save depth image
        # save_depth_img(responses[2])

        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException as err:
        print(err)
        pass
