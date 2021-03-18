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


class MoveWP():

    def __init__(self):
        #initilizing car controls
        self.throttle = 0 
        self.brake = 0
        self.steering = 0
        self.Done = False

        rospy.loginfo("Initializing waypoint publishers")
        self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)
        self.steerPub = rospy.Publisher('lka/steering', Float64, queue_size=1)
        self.controlPub = rospy.Publisher("request",String,queue_size=1)
        
        self.waypoint_x = [0, 70, 118, 127, 90, 10, 0]
        self.waypoint_y = [0, 0, 0, -119, -130, -130, -5]

        
        self.waypoint = PoseStamped()
        self.waypoint.header = "goal"
        self.next_wp = PoseStamped()
        self.next_wp.header = "next"
        self.i = 0

        self.waypoint.pose.position.x = self.waypoint_x[self.i]
        self.waypoint.pose.position.y = self.waypoint_y[self.i]
        self.wp_x = (self.waypoint.pose.position.x)
        self.wp_y = (self.waypoint.pose.position.y)

        self.next_wp.pose.position.x = self.waypoint_x[self.i+1]
        self.next_wp.pose.position.y = self.waypoint_y[self.i+1]
        self.next_x = (self.next_wp.pose.position.x)
        self.next_y = (self.next_wp.pose.position.y)
        

    def set_action(self):

        self.waypoint.pose.position.x = self.waypoint_x[self.i]
        self.waypoint.pose.position.y = self.waypoint_y[self.i]
        self.wp_x = (self.waypoint.pose.position.x)
        self.wp_y = (self.waypoint.pose.position.y)

        self.next_wp.pose.position.x = self.waypoint_x[self.i+1]
        self.next_wp.pose.position.y = self.waypoint_y[self.i+1]
        self.next_x = (self.next_wp.pose.position.x)
        self.next_y = (self.next_wp.pose.position.y)
        rospy.loginfo('goal reached: {}, {}'.format(self.wp_x, self.wp_y))
        rospy.loginfo('next goal: {}, {}'.format(self.next_x, self.next_y))
        
    def get_action(self):

        if(((self.wp_x < self.next_x) and (self.wp_y == self.next_y)) or ((self.wp_x == self.next_x) and (self.wp_y < self.next_y))):
            self.action = "STRAIGHT"
        elif((self.wp_x < self.next_x) and (self.wp_y < self.next_y)):
            self.action = "LEFT"    
        elif((self.wp_x < self.next_x) and (self.wp_y > self.next_y)):
            self.action = "RIGHT"
        return self.action

    def cll(self, data):
        print("CLL:",self.Done)
        if data == String("1"):
            self.throttlePub.publish(self.throttle)
            self.brakePub.publish(self.brake)
            self.steeringPub.publish(self.steering)
            self.Done = True
            self.i = self.i+1

    def control(self):
        rospy.Subscriber("controller",String,cll)

    def set_controls(self, msg):
            rospy.loginfo(msg.data)
            self.reached = msg.data
            if (self.reached == "Reached"):
                self.get_action()
                rospy.loginfo(self.action)
                if self.action == "STRAIGHT":
                    return
                else:
                    if self.action == "LEFT":
                        self.throttle = 0.9
                        self.steering = -1
                        self.brake = 0
                    elif self.action == "RIGHT":
                        self.throttle = 0.9
                        self.steering = 1
                        self.brake = 0
                    rospy.loginfo("turning")
                    requestClearance = "1"
                    self.controlPub.publish(requestClearance)
                    self.control()
                    print("Control",self.Done)
                    reset = str("11")
                    self.controlPub.publish(reset)
                    print("XXXXX",self.Done)
                    if self.Done == True:
                        reset = str("11")
                        self.controlPub.publish(reset)
                        self.Done = False
                    rospy.loginfo('Throttle: {} Brake: {} Steer: {}'.format(self.throttle, self.brake, self.steering))


def listener():
    move = MoveWP()

    #waypoint node
    rospy.loginfo("Initializing waypoint node")
    rospy.init_node('waypoint', anonymous=True)
    # Connect to AirSim
    rospy.loginfo("Connecting airsim")
    client = airsim.CarClient()
    client.confirmConnection
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rospy.loginfo("Waiting")
        rospy.Subscriber("waypoint",String, move.set_controls)
        rate.sleep()

# class for trajectory path planner
if __name__ == "__main__":
    listener()
    