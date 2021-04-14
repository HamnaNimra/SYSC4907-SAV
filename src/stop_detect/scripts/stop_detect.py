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

Done = False
pub = rospy.Publisher("request/msg",String,queue_size=10)

class Dn():
    def __init__(self):
        self.brakePub = rospy.Publisher('acc/brake',Float64,queue_size=1)
        self.tPub = rospy.Publisher('acc/throttle',Float64,queue_size=1)

def cll(data):
    global Done
    b=Dn()
    if data == String("3"):
        b.tPub.publish(-1)
        b.brakePub.publish(1)
        Done = True

def control():
    sub = rospy.Subscriber("controller",String,cll)

class DetectStopSign():

    def __init__(self):
        self.brakePub = rospy.Publisher('acc/brake',Float64,queue_size=1)
        self.tPub = rospy.Publisher('acc/throttle',Float64,queue_size=1)

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
        bluecnts = cv.findContours(mask.copy(), cv.RETR_EXTERNAL,cv.CHAIN_APPROX_SIMPLE)#[-2]        
        bluecnts = bluecnts[1] if imutils.is_cv2() else bluecnts[0]
        if bluecnts is None:
            bluecnts = []

        if len(bluecnts) > 0:
            blue_area = max(bluecnts, key=cv.contourArea)
            (x,y,w,h) = cv.boundingRect(blue_area)
            cv.rectangle(image,(x,y),(x+w, y+h),(0,255,0),2)
             # Canny on region of image
            text = image[y:y+h, x:x+w]
            a = imutils.resize(text, width=min(85, image.shape[1]))
            l = cv.Canny(a,100,295)
            cv.waitKey(5)
            # Uncomment next 2 line for troubleshooting
            #cv.imshow("Canny",l)
            #cv.imshow("Text",a)

            # Use line 100 to 110 if running on windows machine else use line 114 to 128
            avg = np.average(a, axis=0)
            ac = np.average(avg, axis=0)
            if ac[0] > 15 and ac[1]>50 and ac[2] > 15 and ac[0] < 30 and ac[1] <70 and ac[2]< 30:
                requestClearance = "3"
                pub1 = rospy.Publisher("request",String,queue_size=1)
                pub1.publish(requestClearance)
                control()
                time.sleep(1)
                if Done == True:
                    reset = str("13")
                    pub1.publish(reset)


        # Use line 114 to 128 if running on Linux else use line 100 tp 110
        #text = pytesseract.image_to_string(l)
        #rospy.loginfo(text)
        #if re.search('[a-zA-Z]', text):
        #if text:
            # Show Image Processing Results
            #cv.imshow("Text",l)
            #cv.imshow("img_hsv",image)
            #pub = rospy.Publisher("request",String,queue_size=1)
            #requestClearance = "3"
            #pub.publish(requestClearance)
            #control()
            #if Done == True:
                #time.sleep(5)
                #reset = str("13")
                #pub.publish(reset)

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


# Test functions to metrics of the module
def test_DetectStopSign():
    # Testing the Functionality of the Detect Stop Sign Function
    detect_stop = DetectStopSign()
    detect_stop.detectAndDisplay
    if detect_stop == True:
        print("Function worked as Expected")
    else:
        print("Function failed")

def test_Subsumption_DetectStopSign():
    # Unit test the Control Function:

    # Simulatie that a stop sign is detected by module
    pub = rospy.Publisher("request",String,queue_size=1)
    requestClearance = "3"
    pub.publish(requestClearance)
    control()
    if Done == True:
        reset = str("13")
        pub1.publish(reset)
        return 0
    else:
        return 1

    takeControl = control()
    if takeControl == 1:
        print("Exit Code", takeControl)
        print("Communication with Subsumption worked as expceted")
    else:
        print("Exit Code", takeControl)
        print("Communication with Subsumption FAILED")


def listener():
    rospy.init_node('object_detect', anonymous=True)
    avoid_ped = AvoidPedestrians()
    detect_ped = DetectPedestrians(avoid_ped)
    detect_stop = DetectStopSign()
    rospy.Subscriber('airsim/image_raw', Image, detect_stop.detectAndDisplay)
    rospy.spin()

    while True:
        data_car1 = client.getDistanceSensorData(vehicle_name="Car1")
        data_car2 = client.getDistanceSensorData(vehicle_name="Car2")
        rospy.loginfo("Distance sensor data: Car1: {data_car1.distance}, Car2: {data_car2.distance}")
    client = airsim.CarClient()
    client.confirmConnection()

if __name__ == "__main__":
    listener()
