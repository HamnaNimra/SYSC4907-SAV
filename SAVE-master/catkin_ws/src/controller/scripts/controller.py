#!/usr/bin/env python
from controls.msg import Control
from threading import Semaphore
from std_msgs.msg import String

import rospy
import airsim
import numpy
import pprint
import time
import threading

# Obstacle Detection 5
# ACC Module 4
# STOP SIGN Detection 3
# LKA SModule 2
# Geo-Mapping 1
x = 0


request = None
pub = rospy.Publisher("controller/clearance",String, queue_size=10)

def callback(data):
    request = data
    print("Request:",request)

def getPermission(r):
    clearance_level = r
    # This Function will get all the commands and allow control
    obstacle_detection = String("5")
    acc_module = String("4")
    stopsign_detection = String("3")
    lka_module = String("2")
    geo_module = String("1")
    reset = String("10")

    global x  
    if clearance_level == obstacle_detection:
        if x == 0:
            x = 1
            print("Lock Closed")
            key = str(5)
            pub.publish(key)
            print ("ACCESS SENT Obstacle Detection")
    if clearance_level == acc_module:
        if x == 0:
            x = 1
            print("Lock Closed")
            key = str(4)  
            pub.publish(key)
            print ("ACCESS SENT ACC")
    if clearance_level == stopsign_detection:
        if x  == 0:
            x = 1
            print("Lock Closed")
            key = str(3)
            pub.publish(key)
            print ("ACCESS SENT Stop Sign Module")
    if clearance_level == lka_module:
        if x == 0:
            x = 1
            print("Lock Closed")
            key = str(2)
            pub.publish(key)
            print ("ACCESS SENT LKA")
    if clearance_level == geo_module:
        if x == 0:
            x = 1
            print("Lock Closed")
            key = str(1)
            pub.publish(key)
            print ("ACCESS SENT GEO Module")
    if clearance_level == reset:
        x = 0
        print("Lock Open")


def listener():
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber("request/msg",String, getPermission)
    while not rospy.is_shutdown():
        getPermission(request)
        rospy.sleep(1)

    #rate=rospy.Rate(10)

if __name__ == "__main__":
    listener()



