#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from mapping_navigation.msg import PathData
from geometry_msgs.msg import PoseStamped


class Navigation:

    """
    This does mapping
    """

    def __init__(self):
        self.steering_pub = rospy.Publisher('steering', Float64, queue_size=10)
        self.curve_pub = rospy.Publisher('curve_data', String, queue_size=10)
        self.path_data = rospy.Publisher('pathData', PathData, queue_size=10)

    def listener(self):
        rospy.init_node('talker', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.spin()

    def handle_gps_data(self, data):
        self.steering_pub.publish(1)
        self.curve_pub.publish('curve_data')
        self.path_data.publish(PathData())
        print('Handles gps data')


if __name__ == "__main__":
    mapping = Navigation()
    mapping.listener()
