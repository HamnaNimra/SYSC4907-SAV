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
# CLass for Stop Sign Detection


pub = rospy.Publisher("request/msg",String,queue_size=10)

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
        time.sleep(5)
        reset = "10"
        pub.publish(reset)
        print("Reset SENT")

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
        car_controls.brake = 1
        client.setCarControls(car_controls)
        time.sleep(0.5)
        client.enableApiControl(False)
        time.sleep(0.5)

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

        print (text)
        if re.search('[a-zA-Z]', text):
            # Show Image Processing Results
            #cv.imshow("Text",l)
            cv.imshow("img_hsv",image)
            
            # New Subsumption Code
            rospy.init_node('obstacle_avoid')
            pub = rospy.Publisher("request",String,queue_size=1)
            requestClearance = "5"
            pub.publish(requestClearance)
            time.sleep(1)
            control()
            reset = str("10")
            pub.publish(reset)
            time.sleep(1)



            # Old Method
            #getControl()
            #requestControl()
"""
Class for pedestrian detection 
"""      
class DetectPedestrians():

    def __init__(self, avoid_class):
        self.hog = cv.HOGDescriptor()
        self.hog.setSVMDetector(cv.HOGDescriptor_getDefaultPeopleDetector())
        self.avoid = avoid_class
        
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
        image = imutils.resize(image, width=min(400, image.shape[1]))
        # detect people in the image
        # winStride indicates the step size for the boxes that search the image
        # padding indicates the number of pixels added to pad the sliding boxes 
        # scale indicates the scale of the image 
        (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4),
            padding=(8, 8), scale=1.05)

        # apply non-maxima suppression to the bounding boxes using a
        # fairly large overlap threshold to try to maintain overlapping
        # boxes that are still people
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)
        # # draw the final bounding boxes
        # for (xA, yA, xB, yB) in pick:
        #     cv.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)
        # # display image
        # cv.imshow("After NMS", image)
        # cv.waitKey(1)

        self.avoid.objects_in_road(pick)

## ==============================================================================================================================================================================

def listener():
    #global detect_ped 
    #global avoid_ped 
    rospy.init_node('object_detect', anonymous=True)
    avoid_ped = AvoidPedestrians()
    detect_ped = DetectPedestrians(avoid_ped)
    detect_stop = DetectStopSign()
    #rospy.Subscriber('airsim/image_raw', Image, detect_ped.detectAndDisplay)
    rospy.Subscriber('airsim/image_raw', Image, detect_stop.detectAndDisplay)
    #rospy.Subscriber('lka/lanes', Lanes, detect_ped.avoid.get_lines)
    rospy.spin()


    while True:
        data_car1 = client.getDistanceSensorData(vehicle_name="Car1")
        data_car2 = client.getDistanceSensorData(vehicle_name="Car2")
        
        rospy.loginfo("Distance sensor data: Car1: {data_car1.distance}, Car2: {data_car2.distance}")
        time.sleep(1.0)
    client = airsim.CarClient()
    client.confirmConnection()

if __name__ == "__main__":
    listener()
