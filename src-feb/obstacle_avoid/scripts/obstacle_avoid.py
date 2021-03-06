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

#pub = rospy.Publisher("request",String,queue_size=1)

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

    def callback(r):
        print("Permission :",r)
        if data == String("5"):
            print ("ACCESS GRANTED")
            time.sleep(2)
            reset = str("10")
            pub.publish(reset)

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
                if objDistance < 15:
                    #cll(String("5"))
                    rospy.init_node('obstacle_avoid')
                    pub = rospy.Publisher("request",String,queue_size=1)
                    requestClearance = "5"
                    pub.publish(requestClearance)
                    time.sleep(1)
                    control()
                    reset = str("10")
                    pub.publish(reset)
                    time.sleep(1)
                    #BS
                    #sub = rospy.Subscriber("controller",String,self.callback)
                    #print("Sub Reached")
                    #rospy.spin()
                    # Working Control Code
                    #self.client.enableApiControl(True)
                    #car_controls = airsim.CarControls()
                    #car_controls.steering = -1
                    #car_controls.throttle = 1
                    #self.client.setCarControls(car_controls)
                    #time.sleep(2.0)
                    #self.client.enableApiControl(False)
                    # Stop the car

                    #requestControl()
def cll(data):
    global pub
    client = airsim.CarClient()
    state = client.getCarState()
    client.confirmConnection() 
    print(data)
    if data == String("5"):
        #print ("ACCESS GRANTED")
        # STOP THE CAR 
        client.enableApiControl(True)
        car_controls = airsim.CarControls()
        car_controls.steering = -1
        car_controls.throttle = 1
        client.setCarControls(car_controls)
        time.sleep(0.7)
        client.enableApiControl(False)
        time.sleep(0.5)

def control():
    sub = rospy.Subscriber("controller",String,cll)


def requestControl():
    global pub 
    # Request Permission to take control over car
    rospy.init_node('obstacle_avoid')#, anonymous=True)
    rate = rospy.Rate(1)
    requestClearance = "5"
    pub.publish(requestClearance)
    print("DONE!!!")

# CLass for Stop Sign Detection
def listener():
    sensorTest = DistanceTest()
    while (True):
        sensorTest.execute()
        time.sleep(0.2)

if __name__ == "__main__":
    listener()
