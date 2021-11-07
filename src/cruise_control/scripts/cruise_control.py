#!/usr/bin/env python3

import rospy
from mapping_navigation.msg import PathData
from sensor_msgs.msg import PointCloud
from std_msgs.msg import Float64


class CruiseControl:

    def __init__(self):
        self.steeringPub = rospy.Publisher("steering", Float64, queue_size = 10)
        self.brakingPub = rospy.Publisher("braking", Float64, queue_size = 10)
        self.throttlePub = rospy.Publisher("throttling", Float64, queue_size = 10)

    def listener(self):
        rospy.init_node("cruise_control", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        # Define Path data structure
        rospy.Subscriber("navigation", PathData, self.handle_path_data)
        rospy.spin()

    def handle_lidar_data(self):
        print("Obtained lidar data")

    def handle_path_data(self):
        print("Obtained path data")

    def publish_results(self):
        rate = rospy.Rate(2)
        while rospy.is_shutdown():
            self.steeringPub.publish(1)
            self.brakingPub.publish(1)
            self.throttlePub.publish(1)
            rate.sleep()


if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
    cc.publish_results()
