#!/usr/bin/env python3

# Example ROS node for publishing AirSim images.

import airsim

import rospy

# ROS Image message
from sensor_msgs.msg import Image, CompressedImage
from typing import List

def airpub():
    pub = rospy.Publisher("airsim/image_raw", Image, queue_size=1)
    pub2 = rospy.Publisher("airsim/image_uint8_png",  CompressedImage, queue_size=1)
    rospy.init_node('image_raw', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    # connect to the AirSim simulator 
    host_ip = rospy.get_param('/host_ip')
    client = airsim.CarClient(ip=host_ip)
    client.confirmConnection()

    while not rospy.is_shutdown():
        # get camera images from the car
        responses: List[airsim.ImageResponse] = client.simGetImages([
            # png format
            airsim.ImageRequest(0, airsim.ImageType.Scene),
            # scene vision image in uncompressed RGB array
            airsim.ImageRequest("1", airsim.ImageType.Scene, False, False)])

        img_rgb_string = responses[1].image_data_uint8
        # TODO: write image_data_uint8 to a file?
        rospy.loginfo(responses[0])

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

        # Publish compressed image
        cmp_img = CompressedImage()
        cmp_img.header.stamp = rospy.Time.now()
        cmp_img.format = 'png'
        cmp_img.data = responses[0].image_data_uint8

        # log time and size of published image
        rospy.loginfo(len(responses[1].image_data_uint8))
        # publish image message
        pub.publish(msg)
        pub2.publish(cmp_img)
        # sleep until next cycle
        rate.sleep()


if __name__ == '__main__':
    try:
        airpub()
    except rospy.ROSInterruptException as err:
        print(err)
        pass
