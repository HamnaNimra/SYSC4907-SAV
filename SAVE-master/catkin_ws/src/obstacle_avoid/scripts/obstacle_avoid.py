#!/usr/bin/env python
from controls.msg import Control
from threading import Semaphore
from std_msgs.msg import Float64
from std_msgs.msg import String


import rospy
import airsim
import numpy
import pprint

import time

pub = rospy.Publisher("request/msg",String,queue_size=10)

class DistanceTest:
    def __init__(self):
        # Connect to AirSim
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.throttle = 0 
        self.brake = 0
        self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)

       # self.client.enableApiControl(True)

    def parse_lidarData(self, data):
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
        return points

    def execute(self):
        state = self.client.getCarState()
        distanceData = self.client.getDistanceSensorData();

        for i in range(1,3):
            lidarData = self.client.getLidarData();
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                x =[]
                for i in points[:4]:
                    x.append(i[0])
                objDistance = sum(x) / len (x)
                print (objDistance)
                if objDistance < 10:
                    # Stop the car
                    control()
                    requestControl()
def callback(data):
    global pub
    if data == String("5"):
        print ("ACCESS GRANTED")
        # STOP THE CAR 
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(True)
        car_controls = airsim.CarControls()
        print("STEERING")
        car_controls.brake = 1
        car_controls.steering = -1
        car_controls.throttle = 1
        client.setCarControls(car_controls)
        reset = "10"
        pub.publish(reset)
        print("Reset Sent")
        client.enableApiControl(False)

def control():
    sub = rospy.Subscriber("controller/clearance",String,callback)


def requestControl():
    global pub 
    # Request Permission to take control over car
    rospy.init_node('obstacle_avoid')#, anonymous=True)
    rate = rospy.Rate(10)
    requestClearance = "5"
    pub.publish(requestClearance)

# CLass for Stop Sign Detection
def listener():
    sensorTest = DistanceTest()
    while (True):
        sensorTest.execute()

if __name__ == "__main__":
    listener()
