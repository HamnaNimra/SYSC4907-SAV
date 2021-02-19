#!/usr/bin/env python
import cv2 as cv
import numpy as np
import pytesseract
import rospy
from controls.msg import Control
import re
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import random
from imutils.object_detection import non_max_suppression
import imutils 
import airsim

from std_msgs.msg import Float64

from object_avoid import AvoidPedestrians

from lka.msg import Lane
from lka.msg import Lanes
from std_msgs.msg import String


x = 0

def callback(data):
    global pub
    print(String("3"))
    if data == String("3"):
        print("ACCESS GRANTED")
        # STOP THE CAR
        client = airsim.CarClient()
        client.confirmConnection()
        client.enableApiControl(True)
        car_controls = airsim.CarControls()
        rospy.loginfo("STOP")
        car_controls.brake = 1
        client.setCarControls(car_controls)
        reset = "10"
        pub.publish(reset)

def cll(data):
    global x
    client = airsim.CarClient()
    state = client.getCarState()
    client.confirmConnection() 
    print(data)
    if data == String("3"):
        # STOP THE CAR 
        client.enableApiControl(True)
        car_controls = airsim.CarControls()
        car_controls.brake = 1
        client.setCarControls(car_controls)
        time.sleep(2)
        client.enableApiControl(False)
        print("CLL:",x)
        x = 1

def control():
    sub = rospy.Subscriber("controller",String,cll)

def requestControl():
    #Request Persmission to take Control over car
    #rospy.init_node('stopSign_Detect')#, anonymous=True)
    global pub
    rate = rospy.Rate(10)
    requestClearance = str("3")
    pub.publish(requestClearance)


class DetectStopSign():

    def __init__(self):
        self.brakePub = rospy.Publisher('object_avoid/brake',Float64,queue_size=1)

    def getImage(self, img):
        bridge  = CvBridge()

        try:
            cv_img = bridge.imgmsg_to_cv2(img, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.loginfo(e)
        
        return cv_img

    def processImage(self, image):
        image_grey = cv.cvtColor(image, cv.COLOR_BGR2GRAY)
        image_grey = cv.equalizeHist(image_grey)
        return image_grey
 
       
    def detectAndDisplay(self, img):
        image = self.getImage(img)
        # resize image to improve the accuracy and decrease detection time 

        # converting from BGR to HSV color space
        img_hsv=cv.cvtColor(image, cv.COLOR_BGR2HSV)

        #image = cv.Canny(img_hsv,100,200)
        mask1 = cv.inRange(img_hsv, (0,50,20), (5,255,255))
        mask2 = cv.inRange(img_hsv, (175,50,20), (180,255,255))

        mask = cv.bitwise_or(mask1, mask2)
        
        # Making the Boxes around Area of Interest
        bluecnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)[-2]
        blue_area = max(bluecnts, key=cv.contourArea)
        (x,y,w,h) = cv.boundingRect(blue_area)
        cv.rectangle(image,(x,y),(x+w, y+h),(0,255,0),2)
    
        # Canny on region of image
        text = image[y:y+h, x:x+w]
        a = imutils.resize(text, width=min(85, image.shape[1]))
        l = cv.Canny(a,100,295)
        #cv.imshow("Canny",l)
        # For Detecting the TEXT from the stop sign
        cv.waitKey(5)
        # TEMOVE THE COMMENT #
        text = pytesseract.image_to_string(l) 
        rospy.loginfo(text)
        cv.imshow("Text",l)
        cv.imshow("img_hsv",image)

        #if re.search('[a-zA-Z]', text):
        if False:
            #Show Image Processing Results
            #cv.imshow("Text",l)
            #cv.imshow("img_hsv",image)
            
            # New Subsumption Code
            pub = rospy.Publisher("request",String,queue_size=1)
            requestClearance = "5"
            pub.publish(requestClearance)
            time.sleep(1)
            control()
            reset = str("10")
            pub.publish(reset)
            time.sleep(1)

    def test(self,img):
        global x
        time.sleep(2)
        i = random.randint(0,20)
        print("IIIIIIII:", i)
        if i < 3:
            pub = rospy.Publisher("request",String,queue_size=1)
            requestClearance = "3"
            pub.publish(requestClearance)
            control()
            print ("After Control:",x)
            #if x == 1:
            reset = str("11")
            pub.publish(reset)
             #   x = 0


def listener():
    rospy.init_node('object_detect', anonymous=True)
    detect_stop = DetectStopSign()
    #rospy.Subscriber('airsim/image_raw', Image, detect_stop.detectAndDisplay)
    rospy.Subscriber('airsim/image_raw', Image, detect_stop.test)
    rospy.spin()

if __name__ == "__main__":
    listener()
