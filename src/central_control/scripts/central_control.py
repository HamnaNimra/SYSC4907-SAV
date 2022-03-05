#!/usr/bin/env python3
import rospy
import airsim
import numpy as np
import cv2 as cv

from sign_car_recognition.msg import DetectionResult, DetectionResults
from std_msgs.msg import Float64, String
from geometry_msgs.msg import PoseStamped
from lane_keep_assist.msg import LaneStatus, Lane

from enum import Enum
import math
from typing import List, Set, Tuple, Any, Dict


# What state is the car in?
class CCState(Enum):
    NORMAL = 1
    OBJECT_AVOID = 2
    STREET_RULE = 3


class StreetRuleAction(Enum):
    STOP_SIGN = 1
    SPEED_LIMIT = 2  # Unused


class AvoidanceState(Enum):
    DANGER = 1
    WARNING = 2
    NORMAL = 3


class AvoidDirection(Enum):
    """
    If in object avoidance mode, approx. where is the object?
    TODO: Super dumb/basic implementation that is meant to be replaced later
    Must handle multiple objects
    """
    LEFT = 1
    FRONT = 2
    RIGHT = 3


class DetectionRegion(Enum):
    """The values that map to these will likely be heuristic"""
    FAR_LEFT = 1
    LEFT = 2
    CENTER = 3
    RIGHT = 4
    FAR_RIGHT = 5


class CarAction(Enum):
    STOP = 1


class SensorSystem(Enum):
    LIDAR = 1
    CAMERA_DETECTION = 2
    LANE_DETECTION = 3
    NAVIGATION = 4


class CentralControl:
    """Central Controller containing logic that processes all the sensor data"""
    def __init__(self):
        host_ip = rospy.get_param('/host_ip')
        self.client = airsim.CarClient(ip=host_ip)
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.car_controls = airsim.CarControls()

        self.ready = False
        self.speed: float = 0
        self.cc_state: Set[CCState] = {CCState.NORMAL}

        # Store detected sign and object data. Should refresh every object-detect cycle
        self.sign_data: List[DetectionResult] = []
        self.object_data: List[DetectionResult] = []

        self.avoid: List[DetectionResult] = []
        # Action, Data
        self.persistent_actions: List[Tuple[CarAction, Any]] = []
        # Store Lane Keep Assist lane detections
        self.lka_lanes: List[Lane] = []

        # TODO: implement system
        # Each subsystem will have their own formatted recommendation object?
        self.recommendations: Dict[SensorSystem: List] = {
            SensorSystem.LIDAR: [],
            SensorSystem.LANE_DETECTION: [],
            SensorSystem.CAMERA_DETECTION: [],
            SensorSystem.NAVIGATION: []
        }

        self.tick = 0

    # Get scene image using Airsim API. For debugging purposes
    def get_scene_image(self):
        resp: airsim.ImageResponse = self.client.simGetImages(
            [airsim.ImageRequest('0', airsim.ImageType.Scene, False, False)])[0]

        img1d = np.frombuffer(resp.image_data_uint8, dtype=np.uint8)
        # reshape array to 3 channel image array
        return img1d.reshape(resp.height, resp.width, 3)

    def listen(self):
        rospy.init_node("central_control", anonymous=True)
        rospy.Subscriber("lane_info", LaneStatus, self.handle_lane_data)
        rospy.Subscriber("steering", Float64, self.handle_steering_data)
        rospy.Subscriber("braking", Float64, self.handle_breaking_data)
        rospy.Subscriber("throttling", Float64, self.handle_throttling_data)
        rospy.Subscriber("object_detection", DetectionResults, self.handle_object_recognition)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed)

        # Midpoint of the image width
        MID_X = 960 / 2

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # TODO: some intelligent decision making process here?
            self.avoid.clear()

            # Get image to draw on
            scene = self.get_scene_image()

            # Iterate through the stored objects and signs (only stop signs for now)
            # to see if there are actions worth taking
            for sign in self.sign_data:
                # Heuristic value. Not based on real science
                if sign.depth <= 15:
                    self.cc_state.add(CCState.STREET_RULE)

            if not self.object_data:
                self.cc_state.add(CCState.NORMAL)
            else:
                # Lane boundary calculations (left, right)
                # TODO: take from LKA
                # slope = (y1-y2)/(x1-x2)
                # intercept = y - ax
                l_slope, l_int = -0.19868, 364
                r_slope, r_int = 0.27991, 137.28205

                for obj in self.object_data:
                    is_danger = False
                    # Get x-center, y near the bottom of bounding box
                    cx, cy = (obj.xmax + obj.xmin)/2, 0.7*(obj.ymax - obj.ymin) + obj.ymin
                    # Check if in danger zone
                    if cy > l_slope * cx + l_int and cy > r_slope * cx + r_int and obj.depth < 15:
                        is_danger = True
                        self.cc_state.add(CCState.OBJECT_AVOID)
                        self.avoid.append(obj)
                        rospy.loginfo(f'Mark as avoid: {obj}')

                    # Add bounding boxes for debug image
                    # BGR
                    color = (0, 0, 255) if is_danger else (0, 255, 0)
                    x1, x2, y1, y2 = math.floor(obj.xmin), math.floor(obj.xmax), math.floor(obj.ymin), math.floor(obj.ymax)
                    cv.rectangle(scene, (x1, y1), (x2, y2), color, 2)
                    cv.putText(scene, f'{obj.name}: {obj.depth}', (x2+10, y2), 0, 0.3, color)

            # Change state if there's nothing worth avoiding
            if not self.avoid:
                self.cc_state.discard(CCState.OBJECT_AVOID)

            if CCState.OBJECT_AVOID in self.cc_state:
                # Actions taken when in Object Avoidance mode
                # Sort for closest object
                self.avoid.sort(key=lambda x: x.depth)
                # Get closest
                cur = self.avoid[0]
                cx, cy = (math.floor((cur.xmax + cur.xmin)/2), math.floor((cur.ymax + cur.ymin)/2))
                # Take action based on where the object approximately is
                if cx > MID_X:
                    # Right side
                    self.car_controls.steering = -0.8
                    # rospy.loginfo(f'Avoiding to left: {cur}')
                    cv.putText(scene, 'Go Left', (cx, cy), 0, 0.3, (0, 255, 255))
                else:
                    # Left side
                    self.car_controls.steering = 0.8
                    # rospy.loginfo(f'Avoiding to right: {cur}')
                    cv.putText(scene, 'Go Right', (cx, cy), 0, 0.3, (0, 255, 255))
            elif CCState.STREET_RULE in self.cc_state:
                # Actions taken when in Street Rule mode
                pass

            # Set controls
            if self.ready:
                self.client.setCarControls(self.car_controls)

                # Mark danger zones
                cv.line(scene, (0, 364), (453, 274), (0, 0, 255), 3)  # Left
                cv.line(scene, (959, 406), (492, 275), (0, 0, 255), 3)  # Right
                # Write debug image
                if self.tick % 2 == 0:
                    cv.imwrite(f'/home/mango/test_imgs/n_{rospy.Time.now()}_s.png', scene)

            rate.sleep()
            self.tick += 1

    # def control(self):
    #     print("Control loop")

    def handle_lane_data(self, lane_data: LaneStatus):
        if lane_data.gradient_diff < lane_data.hls_diff:
            # Gradient is more reliable
            self.lka_lanes = lane_data.gradient_lanes
        else:
            # HLS color thresholding is more reliable
            self.lka_lanes = lane_data.hls_lanes

    def handle_speed(self, speed: Float64):
        self.speed = speed

    def handle_steering_data(self, steering_data):
        self.car_controls.steering = steering_data.data

    def handle_breaking_data(self, braking_data):
        pass

    def handle_throttling_data(self, throttling_data: Float64):
        self.car_controls.throttle = throttling_data.data

    def handle_object_recognition(self, res: DetectionResults):
        # Determine the "region" that the object falls into (all in front)
        # far_left/left/center/right/far_right
        # TODO: use angle in future?
        if not self.ready:
            self.ready = True

        # Sort through and deal with detections
        self.sign_data.clear()
        self.object_data.clear()

        detection_list: List[DetectionResult] = res.detection_results
        for detection in detection_list:
            if detection.class_num == 11 and detection.depth < 40 and detection.confidence > 0.4:
                # Stop sign. Depth is likely going to be smaller than 25 because of the image resolution
                self.sign_data.append(detection)
            elif detection.class_num == 2 and detection.confidence > 0.4:
                # Only deal with cars for now
                self.object_data.append(detection)


if __name__ == "__main__":
    # Do something
    centralControl = CentralControl()
    centralControl.listen()

