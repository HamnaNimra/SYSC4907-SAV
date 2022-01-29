#!/usr/bin/env python3

import rospy
import lane_turn
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud, Image
from mapping_navigation.msg import Steer


class LaneKeepAssist:
    """
    Keeps them lanes
    """

    def __init__(self):
        #self.steering_pub = rospy.Publisher('steering', Float64, queue_size=10)
        self.steering_pub_test = rospy.Publisher('steering_test', Steer, queue_size=10)
        self.lanes = None
        self.new_steering_input = 0
        self.curr_steering_input = 0
        self.max_angle_deviation = 0.1
        self.initial_angle = False

    def listener(self):
        rospy.init_node('LaneKeepAssist', anonymous=True)
        rospy.Subscriber("airsim/image_raw", Image, self.handle_camera_data)
        # rospy.Subscriber("carData", String, self.handle_car_data)
        # rospy.Subscriber("airsimPose", PoseStamped, self.handle_gps_data)
        # rospy.Subscriber("lidar", PointCloud, self.handle_lidar_data)
        # rospy.Subscriber("curve_data", String, self.handle_curve_data)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            # Ensures that the angle cannot change too much each run.
            if self.initial_angle:
                angle_deviation = self.new_steering_input - self.curr_steering_input
                if abs(angle_deviation) > self.max_angle_deviation:
                    self.curr_steering_input = int(
                        self.curr_steering_input + self.max_angle_deviation * (angle_deviation / abs(angle_deviation)))
                    rospy.loginfo(f'too much deviation {angle_deviation}')
                else:
                    self.curr_steering_input = self.new_steering_input

            rospy.loginfo(f'Steering angle {self.new_steering_input}')
            rospy.loginfo(f'Lanes {self.lanes}')
            # self.steering_pub.publish(self.new_steering_input)
            steering_message = Steer()
            steering_message.angle = self.new_steering_input
            steering_message.system = 'LKA'
            self.steering_pub_test.publish(steering_message)
            rate.sleep()

    # Handles the camera data
    def handle_camera_data(self, image):
        self.lanes, steering_input, error = lane_turn.process_image(image)
        if steering_input:
            self.new_steering_input = steering_input / 45
            self.initial_angle = True

        if error:
            rospy.loginfo(error)

    # Handles the GPS data
    # def handle_gps_data(self, data):
    #     print("Got gps data")

    # Handles the car data
    # def handle_car_data(self, data):
    #     print("Got car data")

    # Handles the lidar data
    # def handle_lidar_data(self, data):
    #     print("Got lidar data")

    # Handles the curve data
    # def handle_curve_data(self, data):
    #     print("Got curve data")

if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()
