#!/usr/bin/env python3

import rospy
import lane_turn
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud, Image
from lane_keep_assist.msg import LaneStatus, Lane


class LaneKeepAssist:

    def __init__(self):
        self.steering_pub_test = rospy.Publisher('lane_info', LaneStatus, queue_size=10)
        self.gradient_state = 1
        self.hls_state = 1
        self.segmentation_state = 1
        self.gradient_lanes = []
        self.hls_lanes = []
        self.segmentation_lane = []
        self.gradient_percent_diff = 0
        self.hls_percent_diff = 0
        self.image = None
        self.segmented_image = None
        self.road_colour = 0
        self.first_detection = True

    def listener(self):
        rospy.init_node('LaneKeepAssist', anonymous=True)
        rospy.Subscriber("airsim/image_raw", Image, self.handle_camera_data)
        rospy.Subscriber("segmented_image", Image, self.handle_segmented_image)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.image is not None and self.segmented_image is not None:
                self.process()

                # Lane status message carries info on the lane boundaries, road boundaries and the percent diff
                lane_msg = LaneStatus()
                lane_msg.lane_gradient_status = self.gradient_state
                lane_msg.lane_hls_status = self.hls_state
                lane_msg.lane_segmentation_status = self.segmentation_state
                lane_msg.gradient_diff = self.gradient_percent_diff
                lane_msg.hls_diff = self.hls_percent_diff
                lane_msg.gradient_lanes = []
                lane_msg.hls_lanes = []
                lane_msg.segmentation_lane = []

                for lane_result in self.gradient_lanes:
                    lane = Lane()
                    lane.lane = lane_result
                    lane_msg.gradient_lanes.append(lane)

                for lane_result in self.hls_lanes:
                    lane = Lane()
                    lane.lane = lane_result
                    lane_msg.hls_lanes.append(lane)

                for lane_result in self.segmentation_lane:
                    lane = Lane()
                    lane.lane = lane_result
                    lane_msg.segmentation_lane.append(lane)

                self.steering_pub_test.publish(lane_msg)
            rate.sleep()

    # Process the normal scene image and road boundaries image
    def process(self):
        # Unpacking the results
        lane_results, percent_differences, lanes, road_colour = lane_turn.lane_interpret(self.image, self.segmented_image)
        self.gradient_state, self.hls_state, self.segmentation_state = lane_results
        self.gradient_percent_diff, self.hls_percent_diff = percent_differences
        self.gradient_lanes, self.hls_lanes, self.segmentation_lane = lanes
        self.road_colour = road_colour

        # Save the first colour detected for the road detection
        if self.first_detection:
            self.road_colour = road_colour
            self.first_detection = False

        # If the majority colour, aka segment, detection has chaned then the car is no longer detecting the road, it is looking at the wrong segment.
        # Therefore ignore the output from the road detector
        if self.road_colour != road_colour:
            self.segmentation_state = 4
            self.gradient_percent_diff = 1
            self.hls_percent_diff = 1
            self.segmentation_lane = []

        rospy.loginfo(f"Gradient Result: Lane state={self.gradient_state}, Percent Diff={self.gradient_percent_diff}, Lanes={self.gradient_lanes}")
        rospy.loginfo(f"HLS Result: Lane state={self.hls_state}, Percent Diff={self.hls_percent_diff}, Lanes={self.hls_lanes}")
        rospy.loginfo(f"Segmentation Result: Lane state={self.segmentation_state}, Lanes={self.segmentation_lane}\n")

    # Handles the scene camera data
    def handle_camera_data(self, image):
        self.image = image

    # Handles the segmented image data
    def handle_segmented_image(self, segmented_image):
        self.segmented_image = segmented_image


if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()
