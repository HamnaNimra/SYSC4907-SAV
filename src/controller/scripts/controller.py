#!/usr/bin/env python
from controls.msg import Control
from threading import Semaphore
from std_msgs.msg import String

import rospy
import math
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
release = String("0")
global sub
request_queue = []
correctRequest=[String("5"),String("4"),String("3"),String("2"),String("1")]


request = None
pub = rospy.Publisher("controller",String, queue_size=1)


def getPermission(i):
    if i in correctRequest: 
        request_queue.append(i)
        clearance_level = max(request_queue)
        print(clearance_level)
    else: 
        clearance_level = i
    # This Function will get all the commands and allow control
    obstacle_detection = String("5")
    acc_module = String("4")
    stopsign_detection = String("3")
    lka_module = String("2")
    geo_module = String("1")
    reset = String("11")
    global x  
    global release  
    if clearance_level == obstacle_detection:
        if x == 0 and release == String("0"):
            release = String("15")
            x = 1
            key = str(5)
            pub.publish(key)
            print("Control: OBJ")
    if clearance_level == acc_module:
        if x == 0 and release == String("0"):
            release = String("14")
            x = 1
            key = str(4)  
            pub.publish(key)
            print ("Control: ACC")
    if clearance_level == stopsign_detection:
        if x == 0 and release == String("0"):
            release = String("13")
            x = 1
            key = str(3)
            pub.publish(key)
            print ("Control: STOP")
    if clearance_level == lka_module:
        if x == 0 and release == String("0"):
            release = String("12")
            x = 1
            key = str(2)
            pub.publish(key)
            print ("Control: LKA")
    if clearance_level == geo_module:
        if x == 0 and release == String("0"):
            release = String("11")
            x = 1
            key = str(1)
            pub.publish(key)
            print ("ACCESS SENT GEO Module")
    #print("C:", clearance_level)
    #print("RELEASE:",release)
    if clearance_level == release:
        print("CLEARED")
        x = 0
        release = String("0")

def listener():
    rospy.init_node('controller', anonymous=True)
    sub = rospy.Subscriber("request",String, getPermission)
    while not rospy.is_shutdown():
        getPermission(request)
        rospy.sleep(1)

if __name__ == "__main__":
    listener()
