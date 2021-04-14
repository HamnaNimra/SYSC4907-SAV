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

Done = False
class DistanceTest:
    global Done
    def __init__(self):
        # Connect to AirSim
        self.client = airsim.CarClient()
        self.client.confirmConnection()
        self.throttle = 0 
        rospy.init_node('obstacle_avoid')
        self.brake = 0
        self.throttlePub = rospy.Publisher('acc/throttle', Float64, queue_size=1)
        self.brakePub = rospy.Publisher('acc/brake', Float64, queue_size=1)
        self.steeringPub = rospy.Publisher('lka/steering', Float64, queue_size=1)

    def parse_lidarData(self, data):
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))
        return points

    def callback(r):
        if data == String("5"):
            time.sleep(2)

    def execute(self):
        global Done
        state = self.client.getCarState()
        distanceData = self.client.getDistanceSensorData();

        lidarData = self.client.getLidarData();
        if (len(lidarData.point_cloud) < 3):
            print("\tNo points received from Lidar data")
        else:
            points = self.parse_lidarData(lidarData)
            x =[]
            for o in points[:4]:
                x.append(o[0])
            objDistance = sum(x) / len (x)
            print (objDistance)
            if objDistance < 10:
                pub = rospy.Publisher("request",String,queue_size=1)
                requestClearance = "5"
                pub.publish(requestClearance)
                control()
                time.sleep(1)
                reset = str("15")
                pub.publish(reset)
                if Done == True:
                    reset = str("15")
                    pub.publish(reset)
                    Done = False

def cll(data):
    l = DistanceTest()
    global Done
    if data == String("5"):
        l.throttlePub.publish(0.8)
        l.steeringPub.publish(-0.8)
        Done = True

def control():
    sub = rospy.Subscriber("controller",String,cll)

def requestControl():

    global pub 
    # Request Permission to take control over car
    rospy.init_node('obstacle_avoid')#, anonymous=True)
    rate = rospy.Rate(1)
    requestClearance = "5"
    pub.publish(requestClearance)

# Class for Obstacle Sign Detection
def listener():
    sensorTest = DistanceTest()
    while (True):
        sensorTest.execute()

if __name__ == "__main__":
    listener()
