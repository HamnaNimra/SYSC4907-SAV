import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import airsim
import numpy as np
import os
import time
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class Unreal():

    def __init__(self):
        self.publisher = rospy.Publisher('/Car',Marker, queue_size=10)
        self.angle = rospy.Subscriber('/control/control', Vector3, self.controlIn)
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.car_controls = airsim.CarControls()
        self.client.enableApiControl(True)
    
    def controlIn(self, data):
        self.car_controls.throttle = data.x
        self.car_controls.steering = data.y
        rospy.loginfo(data.x)
        rospy.loginfo(data.y)

    def updateCar(self):
        self.client.setCarControls(self.car_controls)
        
    def publishCar(self,event =None):
            
        try:
            self.car_state = self.client.getCarState()
            speed = self.car_state.speed
            self.car_state = self.car_state.kinematics_estimated

            colourSet = ColorRGBA()
            colourSet.a = 1.0
            colourSet.r = 1.0
            colourSet.g = 0.0
            colourSet.b = 0.0

            addingMarker = Marker()
            addingMarker.id = 0
            addingMarker.header.frame_id = 'map'
            addingMarker.pose.position.x = self.car_state.position.x_val*10
            addingMarker.pose.position.y = -(self.car_state.position.y_val -20)*10
            addingMarker.pose.position.z = speed
            addingMarker.pose.orientation.x = self.car_state.orientation.x_val
            addingMarker.pose.orientation.y = self.car_state.orientation.y_val
            addingMarker.pose.orientation.z = self.car_state.orientation.z_val
            addingMarker.pose.orientation.w = -self.car_state.orientation.w_val
            addingMarker.scale.x = 5
            addingMarker.scale.y = 5
            addingMarker.scale.z = 2.5
            addingMarker.action = addingMarker.ADD
            addingMarker.type = addingMarker.CUBE
            addingMarker.color = colourSet

            self.publisher.publish(addingMarker)
            self.updateCar()

        except:
            rospy.loginfo("Lost connection to car, retrying")
            rospy.sleep(1)


if __name__ == '__main__':
    try:
        rospy.init_node('Control_In', anonymous=True)
        unreal = Unreal()

        rospy.Timer(rospy.Duration(0.05), unreal.publishCar)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass