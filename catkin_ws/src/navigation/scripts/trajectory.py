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

        self.req = "1"
        self.release = "11"


        #self.controlPub = rospy.Publisher("request", String, queue_size=1)
        self.waypoint_x = [0, 70, 118.0, 129]
        self.waypoint_y = [0, 1, 1, 121]

        if len(self.waypoint_x) == len(self.waypoint_y) & len(self.waypoint_y) >=2:
            rospy.loginfo("Setting waypoints...")
        else:
            rospy.errinfo("Lengths of waypoint lists are not the same")

        self.waypoint_Id = 0
        self.waypoint = PoseStamped()
        self.waypoint.header = "waypoints"
        
        

    #def set_controls(self, action):
    #    if action == "left":
    #        self.throttle = 0.1
    #        self.steering = -1
    #        self.brake = 0
    #    elif action == "right":
    #        self.throttle = 0.1
    #        self.steering = 1
    #        self.brake = 0
    #    #handle action == straight in execute
    #     
    #    rospy.loginfo('Throttle: {} Brake: {} Steer: {}'.format(self.throttle, self.brake, self.steering))
    #    self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
    #    self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)
    #    self.steerPub = rospy.Publisher('lka/steering', Float64, queue_size=1)
    #    self.controlPub.publish(self.req)
    #    #when task is done, then Done = True
    #    #self.Done = True
    #    self.controlPub.publish(self.release)

    def is_close(self, a, b, tol=1e-0):
        if(abs(a-b) <= tol):
            return True
        else:
            return False
        
    def execute(self, pos):

        self.waypoint.pose.position.x = self.waypoint_x[self.waypoint_Id]
        self.waypoint.pose.position.y = self.waypoint_y[self.waypoint_Id]
        curr_x = int(pos.pose.position.x)
        curr_y = int(pos.pose.position.y)
        self.wp_x = int(self.waypoint.pose.position.x)
        self.wp_y = int(self.waypoint.pose.position.y)
        rospy.loginfo('wp_x: {} wp_y: {} '.format(self.wp_x, self.wp_y))
        x_close = self.is_close((self.wp_x), (curr_x)) 
        y_close = self.is_close((self.wp_y), (curr_y))
        rospy.loginfo('wp_x: {} wp_y: {} curr_x: {} curr_y: {}'.format(self.wp_x, self.wp_y, curr_x, curr_y))
        rospy.loginfo('close_x: {} close_y: {}'.format(x_close, y_close))
        if((x_close == True) and (y_close == True)):
            rospy.loginfo("Goal Reached")

            #if action straight, dont update controls
            #elif right or left
                #update controls
            #update next goal
            self.waypoint_Id = self.waypoint_Id+1
    #def cll(self,data):
    #    if data ==String("1"):

# Class for trajectory path planner

if __name__ == "__main__":
    nav = Navigation()
    rospy.loginfo("Initializing trajectory node")
    #trajectory node
    rospy.init_node('trajectory', anonymous=True)
    rospy.loginfo("Connecting airsim")

    #rospy.loginfo("Initializing trajectory publishers")

    # Connect to AirSim
    client = airsim.CarClient()
    client.confirmConnection()
    while not rospy.is_shutdown():

        rospy.Subscriber("airsimPose", PoseStamped, nav.execute)
        # get state of the car
        #car_state = client.getCarState()
        #pos = car_state.kinematics_estimated.position
        #simPose = PoseStamped()
        #simPose.pose.position.x = pos.x_val
        #simPose.pose.position.y = pos.y_val
        #simPose.pose.position.z = pos.z_val
        #simPose.header.stamp = rospy.Time.now()
        #simPose.header.seq = 1
        #simPose.header.frame_id = "simFrame"
        #nav.execute(simPose)
        rospy.spin()
        #waypoint_x = "[0, 70, 118, 129]"
        #waypoint_y = "[-5, -5, -5, -126]"

        #waypoint_x = waypoint_x.replace("[","").replace("]","")
        #waypoint_y = waypoint_y.replace("[","").replace("]","")
        #waypoint_x = [float(x) for x in waypoint_x.split(",")]
        #waypoint_y = [float(y) for y in waypoint_y.split(",")]
