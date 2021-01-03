#!/usr/bin/env python
from controls.msg import Control
from threading import Semaphore

import rospy
import airsim
import numpy
import pprint

import time


class DistanceTest:
    def __init__(self):
        # Connect to AirSim
        self.client = airsim.CarClient()
        self.client.confirmConnection()
       # self.client.enableApiControl(True)

    def parse_lidarData(self, data):
        points = numpy.array(data.point_cloud, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (int(points.shape[0]/3), 3))

        return points

    def execute(self):
        state = self.client.getCarState()
        distanceData = self.client.getDistanceSensorData();
        #rospy.loginfo("Distance Data_____________________")
        #rospy.loginfo(distanceData.distance)
        if ((distanceData.distance > 1) and (distanceData.distance < 10)):
            car_controls = airsim.CarControls()
            #rospy.loginfo("STOP")
            car_controls.steering = 1
            car_controls.throttle = 1
            time.sleep(2.0)
            self.client.setCarControls(car_controls)

        # Added Stuff
        for i in range(1,3):
            lidarData = self.client.getLidarData();
            if (len(lidarData.point_cloud) < 3):
                print("\tNo points received from Lidar data")
            else:
                points = self.parse_lidarData(lidarData)
                print("DATA")
                print("\tReading %d: time_stamp: %d number_of_points: %d" % (i, lidarData.time_stamp, len(points)))
                print("\t\tlidar position: %s" % (pprint.pformat(lidarData.pose.position)))
                print("\t\tlidar orientation: %s" % (pprint.pformat(lidarData.pose.orientation)))

                f = open("points.csv", "w")
                f.write(lidarData.pose.position)
                f.write(lidarData.pose.orientation)
                f.close()

                time.sleep(5)



def update_controls(controls):
    global car_controls
    rospy.loginfo(controls)
    car_controls.brake = controls.brake
    car_controls.throttle = controls.throttle
    car_controls.steering = controls.steering
    client.setCarControls(car_controls)


# CLass for Stop Sign Detection
def listener():

    rospy.init_node('obstacle_avoid', anonymous=True)
    sensorTest = DistanceTest()

    while (True):
        sensorTest.execute()

#    client = airsim.CarClient()
#    distance_sensor_data = client.getDistanceSensorData(self,"Distance","car")

 #   rospy.loginfo("Distance sensor data: {distance_sensor_data}")

  #  time.sleep(1.0)



if __name__ == "__main__":
    listener()
