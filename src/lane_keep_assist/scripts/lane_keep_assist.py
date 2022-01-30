#!/usr/bin/env python3

import rospy
import lane_turn
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud, Image
from lane_keep_assist.msg import LaneStatus


class LaneKeepAssist:
    """
    Keeps them lanes
    """

    def __init__(self):
        self.steering_pub_test = rospy.Publisher('lane_info', LaneStatus, queue_size=10)
        self.lane_state = 1

    def listener(self):
        rospy.init_node('LaneKeepAssist', anonymous=True)
        rospy.Subscriber("airsim/image_raw", Image, self.handle_camera_data)

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():

            status = LaneStatus()
            status.lane_info = self.lane_state

            self.steering_pub_test.publish(status)
            rate.sleep()

    # Handles the camera data
    def handle_camera_data(self, image):
        self.lane_state = lane_turn.process_image(image)

        if self.lane_state == 1:
            rospy.loginfo("Two Lanes")
        elif self.lane_state == 2:
            rospy.loginfo("One Lane")
        elif self.lane_state == 3:
            rospy.loginfo("One Lane")
        elif self.lane_state == 4:
            rospy.loginfo("No Lanes")


if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()
