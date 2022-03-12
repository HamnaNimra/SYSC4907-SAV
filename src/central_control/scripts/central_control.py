#!/usr/bin/env python3

import rospy
import airsim
from sign_car_recognition.msg import DetectionResult, DetectionResults
from std_msgs.msg import Float64, Float64MultiArray
from lane_keep_assist.msg import LaneStatus, LaneLine
from aabb import AABB
from point import Point


class CentralControl:

    """"
    This is the central controller of the system bla bla
    """
    def __init__(self):
        host_ip = rospy.get_param('/host_ip')

        self.client = airsim.CarClient(ip=host_ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

    def listen(self):
        rospy.init_node("central_control", anonymous=True)
        rospy.Subscriber("lane_info", LaneStatus, self.handle_lane_data)
        rospy.Subscriber("steering", Float64, self.handle_steering_data)
        rospy.Subscriber("braking", Float64, self.handle_breaking_data)
        rospy.Subscriber("throttling", Float64, self.handle_throttling_data)
        rospy.Subscriber("sign_detection", DetectionResults, self.handle_sign_recognition)
        rospy.Subscriber("lidar_data", Float64MultiArray, self.handle_lidar_detection)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.client.setCarControls(self.car_controls)
            rate.sleep()

    def control(self):
        print("Control loop")

    def handle_steering_data(self, steering_data):
        print("Obtained steering data")
        self.car_controls.steering = steering_data.data

    def handle_breaking_data(self, braking_data):
        print("Obtained braking data")

    def handle_throttling_data(self, throttling_data: Float64):
        print("Obtained throttling data")
        self.car_controls.throttle = throttling_data.data

    def handle_sign_recognition(self, detection_results):
        print("Obtained sign recognition data")

    def handle_lidar_detection(self, lidar_data: Float64MultiArray):
        # These are indexes into a bounding box for each of the points that it is composed of
        min_x = 0
        min_y = 1
        min_z = 2
        max_x = 3
        max_y = 4
        max_z = 5
        point_per_aabb = 6

        num_aabb = int(len(lidar_data.data) / point_per_aabb)

        # Find the closest point to the car
        all_aabbs = []

        for x in range(0, num_aabb):
            i = x * point_per_aabb
            min_point = Point(lidar_data.data[i + min_x], lidar_data.data[i + min_y], lidar_data.data[i + min_z])
            max_point = Point(lidar_data.data[i + max_x], lidar_data.data[i + max_y], lidar_data.data[i + max_z])
            aabb = AABB(min_point, max_point)
            all_aabbs.append(aabb)

        # These values can be changed to avoid obstacles that are farther from the car
        car_aabb = AABB(
            min_point=Point(0.0, -1.0, 0.0),
            max_point=Point(10.0, 1.0, 2.0)
        )

        # Find the point of the closest AABB for the car to avoid, if any. Only if an AABB intersects
        # with the car's AABB is an obstacle considering for avoidance

        closest_aabb_vector = None
        distance_to_aabb = None

        for aabb in all_aabbs:
            if car_aabb.intersection(aabb):
                (distance, vector_to_closest_point) = aabb.vector_to_closest_point([0, 0, 0])

                if closest_aabb_vector is None:
                    closest_aabb_vector = vector_to_closest_point
                    distance_to_aabb = distance
                elif distance < distance_to_aabb:
                    closest_aabb_vector = vector_to_closest_point
                    distance_to_aabb = distance

        # Avoid the nearest obstacle if any
        # TODO: Uncomment when intersection notifications are available
        # if closest_aabb_vector is not None:
        #     # Lidar points are relative to the car. The "y" dimension, index 1, refers to left or right.
        #     # The "x" dimension, index 0, refers to the horizontal distance from the car.
        #     if closest_aabb_vector[1] > 0:
        #         self.car_controls.steering = -(min(1.0, 0.125 / closest_aabb_vector[0]))
        #     elif closest_aabb_vector[1] < 0:
        #         self.car_controls.steering = min(1.0, 0.125 / closest_aabb_vector[0])

    # Results from lane detection
    # If the detection methods agree on the number of lane bounds a percent difference per line is returned
    # If the detection methods don't agree a single error of 100 is returned, 100 is the value itself
    # There can be a max number of two lane bounds returned
    def handle_lane_data(self, lane_data):
        print("Obtained lane data")
        rospy.loginfo(f"{lane_data.lane_gradient_status}")
        rospy.loginfo(f"{lane_data.lane_hls_status}")
        rospy.loginfo(f"{lane_data.lane_segmentation_status}")
        rospy.loginfo(f"{lane_data.gradient_lane_bounds}")
        rospy.loginfo(f"{lane_data.hls_lane_bounds}")
        rospy.loginfo(f"{lane_data.segmentation_lane_bounds}")
        rospy.loginfo(f"{lane_data.gradient_diff}")
        rospy.loginfo(f"{lane_data.hls_diff}")



if __name__ == "__main__":
    # Do something
    centralControl = CentralControl()
    centralControl.listen()
