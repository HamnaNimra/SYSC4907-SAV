#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud


class LaneKeepAssist:

    """
    Keeps them lanes
    """
    def __init__(self):
        self.pub = rospy.Publisher('lka/steering',Float64 ,queue_size=10)

    def listener(self):
        rospy.init_node('LaneKeepAssist', anonymous=True)
        rospy.Subscriber("airsim/image_raw", String, self.handle_camera_data)
        rospy.Subscriber("carData", String, self.handle_car_data)
        rospy.Subscriber("airsimPose", PoseStamped, self.handle_gps_data)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        rospy.Subscriber("curve_data", String, self.handle_curve_data)
        rospy.spin()

    #Handles the camera data
    def handle_camera_data(self, data):
        print("Got camera data")

    #Handles the GPS data
    def handle_gps_data(self, data):
        print("Got gps data")

    #Handles the car data
    def handle_car_data(self, data):
        print("Got car data")

    #Handles the lidar data
    def handle_lidar_data(self, data):
        print("Got lidar data")

    #Handles the curve data
    def handle_curve_data(self, data):
        print("Got curve data")

    def publish_results(self):
        rate = rospy.Rate(2) # 2hz
        while not rospy.is_shutdown():
            self.pub.publish(1)
            rate.sleep()


if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()
    lane_keep.publish_results()
