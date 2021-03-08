#!/usr/bin/env python

import time
import setup_path 
import airsim

import rospy
from sensors.msg import Speed
from std_msgs.msg import Float64


def setSpeed():
    pub = rospy.Publisher('sensor/speed', Float64, queue_size=1)
    rospy.init_node('setspeed', anonymous=True)
    rate = rospy.Rate(2) # 2 Hz

    # connect to the AirSim simulator 
    client = airsim.CarClient()
    client.confirmConnection()
    client.enableApiControl(True)
    car_controls = airsim.CarControls()

    while not rospy.is_shutdown():
        car_state = client.getCarState()
        print("Speed %d, Gear %d" % (car_state.speed, car_state.gear))
        # Get the speed of the car

        car_controls.throttle = 1
        car_controls.steering = 1
        client.setCarControls(car_controls)

        # let car drive a bit
        time.sleep(1)

        car_speed = client.getCarState().speed
        rospy.loginfo(car_speed)
        pub.publish(car_speed)
        rate.sleep()


if __name__ == '__main__':
    try:
        setSpeed()
    except rospy.ROSInterruptException:
        pass
