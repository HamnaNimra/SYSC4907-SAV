#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, String
from geometry_msgs.msgs import PoseStamped


class Navigation:

    """
    This does mapping
    """

    def __init__(self):
        self.steering_pub = rospy.Publisher('lka/steering', Float64, queue_size=10)
        self.curve_pub = rospy.Publisher('curveData', String, queue_size=10)

    def listener(self):
        rospy.init_node('talker', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.spin()

    def handle_gps_data(self):
        self.steering_pub.publish(1)
        self.curve_pub.publish('curve_data')
        print('Handles gps data')


if __name__ == "__main__":
    mapping = Navigation()
    mapping.listener()
