#!/usr/bin/env python

# airsim
import airsim
# standard python
import math
import numpy
# ROS
import rospy
# ROS message
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
Done = False
class Navigation:
    global Done
    def __init__(self):
        #trajectory node
        rospy.loginfo("Initializing trajectory node")
        rospy.init_node('trajectory', anonymous=True)

        #initilizing car controls
        self.throttle = 0 
        self.brake = 0
        self.steering = 0

        self.reached = String("Halt")

        rospy.loginfo("Initializing trajectory publishers")
        self.waypointPub = rospy.Publisher("waypoint",String,queue_size=1)
        self.waypoint_x = [0, 70, 118.0, 129]
        self.waypoint_y = [0, 1, 1, 120]

        if len(self.waypoint_x) == len(self.waypoint_y) & len(self.waypoint_y) >=2:
            rospy.loginfo("Setting waypoints...")
        else:
            rospy.errinfo("Lengths of waypoint lists are not the same")

        self.waypoint_Id = 0
        self.waypoint = PoseStamped()
        self.waypoint.header = "waypoints"


    def is_close(self, a, b, tol=1e-0):
        if(abs(a-b) <= tol):
            return True
        else:
            return False

    def execute(self, pos):
        self.reached = String("Halt")
        self.waypointPub.publish(self.reached)
        curr_x = int(pos.pose.position.x)
        curr_y = int(pos.pose.position.y)

        self.waypoint.pose.position.x = self.waypoint_x[self.waypoint_Id]
        self.waypoint.pose.position.y = self.waypoint_y[self.waypoint_Id]
        self.wp_x = int(self.waypoint.pose.position.x)
        self.wp_y = int(self.waypoint.pose.position.y)
        rospy.loginfo('wp_x: {} wp_y: {} '.format(self.wp_x, self.wp_y))
        x_close = self.is_close((self.wp_x), (curr_x)) 
        y_close = self.is_close((self.wp_y), (curr_y))
        rospy.loginfo('wp_x: {} wp_y: {} curr_x: {} curr_y: {}'.format(self.wp_x, self.wp_y, curr_x, curr_y))
        rospy.loginfo('close_x: {} close_y: {}'.format(x_close, y_close))
        if((x_close == True) and (y_close == True)):
            rospy.loginfo("Goal Reached")
            self.reached = String("Reached")
            self.waypoint_Id = self.waypoint_Id+1
            self.waypointPub.publish(self.reached)

# class for trajectory path planner
if __name__ == "__main__":

    nav = Navigation()
    # Connect to AirSim
    rospy.loginfo("Connecting airsim")
    client = airsim.CarClient()
    client.confirmConnection()
    
    rospy.Subscriber("airsimPose", PoseStamped, nav.execute)
    rospy.spin()
        