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
        
        #initilizing car controls
        self.throttle = 0 
        self.brake = 0
        self.steering = 0

        rospy.loginfo("Initializing trajectory publishers")
        self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)
        self.steerPub = rospy.Publisher('lka/steering', Float64, queue_size=1)

        self.waypoint_x = [0, 70, 118.0, 129]
        self.waypoint_y = [0, 1, 1, 120]

        if len(self.waypoint_x) == len(self.waypoint_y) & len(self.waypoint_y) >=2:
            rospy.loginfo("Setting waypoints...")
        else:
            rospy.errinfo("Lengths of waypoint lists are not the same")

        self.waypoint_Id = 0
        self.waypoint = PoseStamped()
        self.waypoint.header = "waypoints"

        self.next_wp = PoseStamped()
        self.next_wp.header = "next"

        #initial action
        self.action = "STRAIGHT"

    def set_controls(self):
        if self.action == "STRAIGHT":
            return
        elif self.action == "LEFT":
            self.throttle = 0.1
            self.steering = -1
            self.brake = 0
        elif self.action == "RIGHT":
            self.throttle = 0.1
            self.steering = 1
            self.brake = 0
        rospy.loginfo('Throttle: {} Brake: {} Steer: {}'.format(self.throttle, self.brake, self.steering))
        global pub = rospy.Publisher("request",String,queue_size=1)
        requestClearance = "1"
        pub.publish(requestClearance)
        control()
        print("Control",Done)
                time.sleep(1)
                reset = str("11")
                pub.publish(reset)
                print("XXXXX",Done)
                if Done == True:
                    reset = str("11")
                    pub.publish(reset)
                    Done = False

    def is_close(self, a, b, tol=1e-0):
        if(abs(a-b) <= tol):
            return True
        else:
            return False

    def get_action(self, x, y):
        
        self.next_wp.pose.position.x = self.waypoint_x[self.waypoint_Id+1]
        self.next_wp.pose.position.y = self.waypoint_y[self.waypoint_Id+1]
        self.next_x = int(self.next_wp.pose.position.x)
        self.next_y = int(self.next_wp.pose.position.y)
        rospy.loginfo(self.next_x, self.next_y)

        if((x < self.next_x) and (y == self.next_y) or (x == self.next_x and y < self.next_y)):
            self.action = "STRAIGHT"
        elif((x < self.next_x) and (y < self.next_y)):
            self.action = "LEFT"
        elif((x < self.next_x) and (y > self.next_y)):
            self.action = "RIGHT"
        rospy.loginfo(action)
        return action

    def execute(self, pos):

        curr_x = int(pos.pose.position.x)
        curr_y = int(pos.pose.position.y)    

        self.waypoint.pose.position.x = self.waypoint_x[self.waypoint_Id]
        self.waypoint.pose.position.y = self.waypoint_y[self.waypoint_Id]
        self.wp_x = int(self.waypoint.pose.position.x)
        self.wp_y = int(self.waypoint.pose.position.y)
        rospy.loginfo('wp_x: {} wp_y: {} '.format(self.wp_x, self.wp_y))
        x_close = self.is_close((self.wp_x), (curr_x)) 
        y_close = self.is_close((self.wp_y), (curr_y))
        get_action(wp_x, wp_y)
        rospy.loginfo('wp_x: {} wp_y: {} curr_x: {} curr_y: {}'.format(self.wp_x, self.wp_y, curr_x, curr_y))
        rospy.loginfo('close_x: {} close_y: {}'.format(x_close, y_close))
        if((x_close == True) and (y_close == True)):
            rospy.loginfo("Goal Reached")
            self.waypoint_Id = self.waypoint_Id+1
            set_controls()

def cll(data):
    n = Navigation()
    global Done
    print("CLL:",Done)
    if data == String("1"):
        n.throttlePub.publish(n.throttle)
        n.brakePub.publish(n.brake)
        n.steeringPub.publish(n.steering)
        Done = True

def control():
    sub = rospy.Subscriber("controller",String,cll)

# Class for trajectory path planner
if __name__ == "__main__":
    nav = Navigation()
    #trajectory node
    rospy.loginfo("Initializing trajectory node")
    rospy.init_node('trajectory', anonymous=True)

    # Connect to AirSim
    rospy.loginfo("Connecting airsim")
    client = airsim.CarClient()
    client.confirmConnection()
    while not rospy.is_shutdown():

        rospy.Subscriber("airsimPose", PoseStamped, nav.execute)
        rospy.spin()
        