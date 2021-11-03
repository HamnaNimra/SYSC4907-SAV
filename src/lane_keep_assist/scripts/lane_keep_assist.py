#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msgs import PoseStamped

class LaneKeepAssist:

    """
    Keeps them lanes
    """
    def __init__(self):

        self.pub = rospy.Publisher('lka/steering',Float64 , queue_size=10)
        rospy.init_node('LaneKeepAssist', anonymous=True)

    def listener(self):
        rospy.Subscriber("airsim/image_raw", String, self.handle_camera_data)
        rospy.Subscriber("carData", String, self.handle_car_data)
        rospy.Subscriber("airsimPose", PoseStamped, self.handle_gps_data)


    def handle_camera_data(self, data):

    def handle_gps_data(self, data):

    def handle_car_data(self, data):

if __name__ == "__main__":
    lane_keep = LaneKeepAssist()
    lane_keep.listener()