#!/usr/bin/env python3

import rospy
import lane_turn
from std_msgs.msg import Float64, UInt8
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud, Image
from lane_keep_assist.msg import LaneStatus


class LaneKeepAssist:

    def __init__(self):
        self.steering_pub_test = rospy.Publisher('lane_info', LaneStatus, queue_size=10)
        self.lane_state = 1
        self.road_state = 1
        self.lane_road_diff = 0
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
                lane_msg.lane_info = self.lane_state
                lane_msg.road_info = self.road_state
                lane_msg.percent_diff = self.lane_road_diff

                self.steering_pub_test.publish(lane_msg)
            rate.sleep()

    # Process the normal scene image and road boundaries image
    def process(self):
        self.lane_state, self.road_state, self.lane_road_diff, slopes, road_colour = lane_turn.lane_interpret(self.image, self.segmented_image)

        # Save the first colour detected for the road detection
        if self.first_detection:
            self.road_colour = road_colour
            self.first_detection = False

        # If the majority colour, aka segment, detection has chaned then the car is no longer detecting the road, it is looking at the wrong segment.
        # Therefore ignore the output from the road detector
        if self.road_colour != road_colour:
            self.road_state = 4
            self.lane_road_diff = 1
            slopes = (slopes[0], 'NAN')

        # Two lanes
        if self.lane_state == 1:
            rospy.loginfo("Lane detection detects two Lanes")
        # One lane Off left
        elif self.lane_state == 2:
            rospy.loginfo("Lane detection detects one Lane")
        # One lane off right
        elif self.lane_state == 3:
            rospy.loginfo("Lane detection detects one Lane")
        # No lanes
        elif self.lane_state == 4:
            rospy.loginfo("Lane detection detects no Lanes")
        rospy.loginfo(f"Result lanes: {self.lane_state}  Result road bounds: {self.road_state}  Percent Difference: {self.lane_road_diff}")
        rospy.loginfo(f"Average slope from lane detector: {slopes[0]} Average slope from road detector: {slopes[1]}")

    # Handles the scene camera data
    def handle_camera_data(self, image):
        self.image = image

    # Handles the segmented image data
    def handle_segmented_image(self, segmented_image):
        self.segmented_image = segmented_image


if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()
