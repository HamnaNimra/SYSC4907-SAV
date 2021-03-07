
import rospy
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
from sensor_msgs.msg import Image # Image is the message type
from sensor_msgs.msg import PointCloud2
import airsim
import matplotlib.pyplot as plt

import cv2
import time
import sys
import math
import numpy as np

outputFile = "cloud.asc" 
color = (0,255,0)
rgb = "%d %d %d" % color
projectionMatrix = np.array([[-0.501202762, 0.000000000, 0.000000000, 0.000000000],
                              [0.000000000, -0.501202762, 0.000000000, 0.000000000],
                              [0.000000000, 0.000000000, 10.00000000, 100.00000000],
                              [0.000000000, 0.000000000, -10.0000000, 0.000000000]])

def array_to_pointcloud2(cloud_arr, stamp=None, frame_id=None):
    '''Converts a numpy record array to a sensor_msgs.msg.PointCloud2.
    '''
    # make it 2d (even if height will be 1)
    cloud_arr = np.atleast_2d(cloud_arr)

    cloud_msg = PointCloud2()

    if stamp is not None:
        cloud_msg.header.stamp = stamp
    if frame_id is not None:
        cloud_msg.header.frame_id = frame_id
    cloud_msg.height = cloud_arr.shape[0]
    cloud_msg.width = cloud_arr.shape[1]
    cloud_msg.fields = dtype_to_fields(cloud_arr.dtype)
    cloud_msg.is_bigendian = False # assumption
    cloud_msg.point_step = cloud_arr.dtype.itemsize
    cloud_msg.row_step = cloud_msg.point_step*cloud_arr.shape[1]
    cloud_msg.is_dense = all([np.isfinite(cloud_arr[fname]).all() for fname in cloud_arr.dtype.names])
    cloud_msg.data = cloud_arr.tostring()
    return cloud_msg 

class videoIn():

    def __init__(self):
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.bridge = CvBridge()
        self.publisher = rospy.Publisher('/Video', Image, queue_size=10)

    def inImage(self, event = None):

        while True:
            rawImage = self.client.simGetImage("0", airsim.ImageType.DepthPerspective)
            
            png = cv2.imdecode(np.frombuffer(rawImage, np.uint8) , cv2.IMREAD_UNCHANGED)
            gray = cv2.cvtColor(png, cv2.COLOR_BGR2GRAY)
            Image3D = cv2.reprojectImageTo3D(gray, projectionMatrix)
            array_to_pointcloud2(Image3D, rospy.Time.now(), '0')
            rospy.loginfo(Image3D)

        # responses = self.client.simGetImages([
        #     # png format
        #     airsim.ImageRequest(0, airsim.ImageType.Scene), 
        #     # uncompressed RGB array bytes
        #     airsim.ImageRequest(1, airsim.ImageType.Scene, False, False),
        #     # floating point uncompressed image
        #     airsim.ImageRequest(1, airsim.ImageType.DepthPerspective, True)])
        # response = responses[2] 

        
        # height = response.height
        # width = response.width

        # output_image = Image()
        # output_image.header.stamp     = rospy.Time.now()
        # output_image.height           = height
        # output_image.width            = width
        # output_image.encoding         = "rgb8"
        # output_image.is_bigendian     = 0
        # output_image.step             = 3 * height

        # output_image.data = response.image_data_uint8

        self.publisher.publish(output_image)


if __name__ == '__main__':
    try:
        rospy.init_node('video_pub_py', anonymous=True)
        video = videoIn()
        rospy.Timer(rospy.Duration(1), video.inImage)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
