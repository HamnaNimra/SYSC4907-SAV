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
        self.speed: Float64 = 0.0

    def listener(self):
        rospy.init_node("cruise_control", anonymous=True)
        rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        # Define Path data structure
        rospy.Subscriber("pathData", PathData, self.handle_path_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)

        # Node is publisher and subscriber- cannot use spin; the publisher methods will never get called

        # Probably should have same rate for this node and speedometer to make them more in sync
        # Higher the rate the smoother the constant speed is, but will take more CPU power
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
         self.publish_results()
         rate.sleep()

    def handle_lidar_data(self, data):
        print("Obtained lidar data")

    def handle_path_data(self, data):
        print("Obtained path data")

    def handle_speed_data(self, speed: Float64):
        rospy.loginfo("Handling the speed data")
        self.speed = speed.data

    def publish_results(self):
        self.steeringPub.publish(1)
        self.brakingPub.publish(1)

        if self.speed < 5:
            # If throttle is too low then car will not go above a certain speed (at least if road is
            # uphill). This value might have to change as a result
            self.throttlePub.publish(0.5)
        else:
            self.throttlePub.publish(0)


if __name__ == "__main__":
    # Do something
    cc = CruiseControl()
    cc.listener()
