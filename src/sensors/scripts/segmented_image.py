#!/usr/bin/env python3

# Example ROS node for publishing AirSim images.

import setup_path
import airsim

import rospy

# ROS Image message
from sensor_msgs.msg import Image


def segmented_image():
    pub = rospy.Publisher("segmented_image", Image, queue_size=1)
    rospy.init_node('segmented_image', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    # connect to the AirSim simulator
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()

    while not rospy.is_shutdown():
        # get camera images from the car
        responses = client.simGetImages([airsim.ImageRequest("1", airsim.ImageType.Segmentation, False, False)])  # scene vision image in uncompressed RGB array

        for response in responses:
            img_rgb_string = response.image_data_uint8

        # Populate image message
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "frameId"
        msg.encoding = "rgb8"
        msg.height = 360  # resolution should match values in settings.json
        msg.width = 640
        msg.data = img_rgb_string
        msg.is_bigendian = 0
        msg.step = msg.width * 3

        # log time and size of published image
        rospy.loginfo(len(response.image_data_uint8))
        # publish image message
        pub.publish(msg)
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        segmented_image()
    except rospy.ROSInterruptException:
        pass
