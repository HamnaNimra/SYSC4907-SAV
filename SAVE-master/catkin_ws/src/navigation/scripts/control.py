import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import airsim
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
import time

# Parameters
k = 2  # look forward gain
Kp_angle = 25 # angular gain
Lfc = 4.0  # [m] look-ahead distance
Kp = 7  # speed proportional gain
dt = 0.1  # [s] time tick
WB = 2  # [m] wheel base of vehicle

show_animation = True


client = airsim.CarClient()
client.confirmConnection()
car_controls = airsim.CarControls()
client.enableApiControl(True)

velocity_data = []
velocity_data.append([])
velocity_data.append([])

Position_Error = []
Position_Error.append([])
Position_Error.append([])
Position_Error.append([])

Angular_Error = []
Angular_Error.append([])
Angular_Error.append([])

Steering_Effort = []
Steering_Effort.append([])
Steering_Effort.append([])


def getTelemetries():
    return client.getCarState()

class State:

    def __init__(self, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))

    def update(self, a, delta):

        car_state = getTelemetries()
        self.x = (car_state.kinematics_estimated.position.x_val -18)*10
        # self.x += self.v * math.cos(self.yaw) * dt
        self.y = -(car_state.kinematics_estimated.position.y_val)*10
        # self.y += self.v * math.sin(self.yaw) * dt
        car_controls.throttle = a * dt

        self.yaw = -car_state.kinematics_estimated.orientation.z_val*2
        # print("Yaw:         {}".format(self.yaw*180/math.pi))
        # print("Delta:       {}".format(delta*180/math.pi))
        # self.yaw += self.v / WB * math.tan(delta) * dt
        self.v = car_state.speed


        # self.v += a * dt
        car_controls.steering = -(self.v / WB) * math.tan(delta) * dt * Kp_angle
        self.rear_x = self.x - ((WB / 2) * math.cos(self.yaw))
        self.rear_y = self.y - ((WB / 2) * math.sin(self.yaw))
        client.setCarControls(car_controls)

        velocity_data[0].append(self.v)
        velocity_data[1].append(rospy.get_time())

        Steering_Effort[0].append(-car_controls.steering)
        Steering_Effort[1].append(rospy.get_time())

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)


class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []
    
    def clear(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def appendState(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)


class PID:
    """PID Controller
    """

    def __init__(self, P=0.2, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback
        .. math::
            u(t) = K_p e(t) + K_i \int_{0}^{t} e(t)dt + K_d {de}/{dt}
        .. figure:: images/pid_1.png
           :align:   center
           Test PID with Kp=1.2, Ki=1, Kd=0.001 (test_pid.py)
        """
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            # dx = [state.rear_x - icx for icx in self.cx]
            # dy = [state.rear_y - icy for icy in self.cy]
            # d = np.hypot(dx, dy)
            # ind = np.argmin(d)
            ind = 0
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else 0
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + Lfc  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                ind = 0
                # not exceed goal
            ind += 1
        if ind == len(self.cx) - 1:
            ind = 0
        return ind, Lf


def pure_pursuit_steer_control(state, trajectory, pind):
    ind, Lf = trajectory.search_target_index(state)

    if pind >= ind:
        ind = pind

    if ind < len(trajectory.cx):
        tx = trajectory.cx[ind]
        ty = trajectory.cy[ind]
    else:  # toward goal
        tx = trajectory.cx[-1]
        ty = trajectory.cy[-1]
        ind = 0
        # ind = len(trajectory.cx) - 1
    yDiff =  ty - state.rear_y
    xDiff = tx - state.rear_x

    Position_Error[0].append(yDiff)
    Position_Error[1].append(xDiff)
    Position_Error[2].append(rospy.get_time())

    alpha = (math.atan2(yDiff, xDiff) - state.yaw)
    if abs(alpha) > math.pi:
        if alpha > 0:        
            Angular_Error[0].append(alpha - math.pi)
        else:
            Angular_Error[0].append(alpha + math.pi)

    else:
        Angular_Error[0].append(alpha)
        
    
    Angular_Error[1].append(rospy.get_time())
    

    delta = math.atan2(2.0 * WB * math.sin(alpha) / Lf, 1.0)

    if ind == len(trajectory.cx) - 1:
        ind = 0
    return delta, ind


class mainPart:

    def __init__(self):

        
        # self.client = airsim.CarClient()
        # self.client.confirmConnection()
        self.control = rospy.Publisher('/control', MarkerArray, queue_size=10)
        self.controlOutPath = rospy.Publisher('/Control_Out_Path', Path, queue_size=10)
        self.path = Path()
        self.pathLoop = 0

        self.path.header.frame_id = 'map'
        self.path.header.stamp = rospy.Time(0)

        self.gotPath = False


        self.cx = []
        self.cy = []
        # initial state
        self.state = State(x=-180, y=0, yaw=0.0, v=0.0)

        data = MarkerArray()
        data = rospy.wait_for_message('/Path_Points', MarkerArray)

        for point in data.markers:
            self.cx.append(point.pose.position.x)
            self.cy.append(point.pose.position.y)
            # rospy.loginfo("x")
            # rospy.loginfo(point.pose.position.x)
            # rospy.loginfo("y")
            # rospy.loginfo(point.pose.position.y)
        

        self.lastIndex = len(self.cx) - 1
        self.time = 0.0
        self.states = States()
        self.states.appendState(self.time, self.state)
        self.target_course = TargetCourse(self.cx, self.cy)
        self.target_ind, _ = self.target_course.search_target_index(self.state)

        # angularControl = PID(1,0,0,)
        self.velocityControl = PID(7,1,1)
        # angularControl.setSampleTime(0.02)
        self.velocityControl.setSampleTime(0.01)

        self.velocityControl.SetPoint = self.target_speed
        self.start_time = rospy.get_time()

    def reset(self):
        self.target_ind = 1
        self.state = State(x=-180, y=0, yaw=math.pi/2, v=0.0)
        self.states.clear()
        self.states.appendState(self.time, self.state)
        self.target_course = TargetCourse(self.cx, self.cy)
        

    def mainLoop(self, event = None):

        carTelemetries = getTelemetries()

        try:

            # carTelemetries = getTelemetries()

            #Angular PID control
            # angularControl.SetPoint = self.target_course.
            # angularControl.update(-carTelemetries.kinematics_estimated.orientation.w_val)
            # di = angularControl.output

            #Velocity PID control
            
            self.velocityControl.update(carTelemetries.speed)
            ai = self.velocityControl.output

            # Calc control input
            # ai = proportional_control(self.target_speed, self.state.v)
            di, self.target_ind = pure_pursuit_steer_control(
                self.state, self.target_course, self.target_ind)

            self.state.update(ai, di)  # Control vehicle

            self.states.append(self.time, self.state)


            if show_animation:  # pragma: no cover

                    outMarkers = MarkerArray()

                    mapMarker = Marker()
                    mapMarker.id = 0
                    mapMarker.header.frame_id = "map"
                    mapMarker.type = mapMarker.SPHERE
                    mapMarker.action = mapMarker.ADD
                    mapMarker.scale.x = 5
                    mapMarker.scale.y = 5
                    mapMarker.scale.z = 5
                    mapMarker.pose.position.z = 0
                    mapMarker.pose.position.x = self.state.x
                    mapMarker.pose.position.y = self.state.y
                    mapMarker.color.a = 1.0
                    mapMarker.color.r = 0.0
                    mapMarker.color.g = 0.0
                    mapMarker.color.b = 1.0
                    mapMarker.lifetime = rospy.Time(5)

                    outMarkers.markers.append(mapMarker)

                    mapMarker = Marker()
                    mapMarker.id = 1
                    mapMarker.header.frame_id = "map"
                    mapMarker.type = mapMarker.SPHERE
                    mapMarker.action = mapMarker.ADD
                    mapMarker.scale.x = 5
                    mapMarker.scale.y = 5
                    mapMarker.scale.z = 5
                    mapMarker.pose.position.z = 0
                    mapMarker.pose.position.x = self.cx[self.target_ind]
                    mapMarker.pose.position.y = self.cy[self.target_ind]
                    mapMarker.color.a = 1.0
                    mapMarker.color.r = 1.0
                    mapMarker.color.g = 0.0
                    mapMarker.color.b = 0.0
                    mapMarker.lifetime = rospy.Time(5)

                    outMarkers.markers.append(mapMarker)

                    self.controlOut.publish(outMarkers)

                    temp = PoseStamped()
                    tempPose = Pose()
                    tempPose.position.z = 0
                    tempPose.position.x = self.state.x
                    tempPose.position.y = self.state.y
                    temp.header.frame_id = "Path: " + str(self.pathLoop)
                    temp.header.stamp = rospy.Time(0)
                    self.pathLoop += 1
                    temp.pose = tempPose
                    self.path.poses.append(temp)
                    self.controlOutPath.publish(self.path)
            
        except:
            self.reset()
            # velocity_data[1] = [x - self.start_time for x in velocity_data[1]]

            # fig, ax = plt.subplots(figsize=(8, 6), dpi=200)
            # ax.set_xlabel("Time (Seconds)")
            # ax.set_ylabel("Velocity (m/s)")
            # ax.grid()
            # ax.plot(velocity_data[1],velocity_data[0], lw=1, color='blue')
            # plt.plot([0, 25], [6, 6], 'k-', lw=1, color='red')
            # fig.savefig('Velocity_Graph.png')

            # Position_Error[2] = [x - self.start_time for x in Position_Error[2]]

            # fig, ax = plt.subplots(figsize=(8, 6), dpi=200)
            # ax.set_xlabel("Time (Seconds)")
            # ax.set_ylabel("Position Difference (m)")
            # ax.grid()
            # ax.plot(Position_Error[2],Position_Error[0], lw=1, color='blue', label='Y Difference')
            # ax.plot(Position_Error[2],Position_Error[1], lw=1, color='red', label='X Difference')
            # ax.legend()
            # fig.savefig('Position_Error_Graph.png')

            # Angular_Error[1] = [x - self.start_time for x in Angular_Error[1]]

            #fig, ax = plt.subplots(figsize=(8, 6), dpi=200)
            #ax.set_xlabel("Time (Seconds)")
            #ax.set_ylabel("Angular Error (Radians)")
            #ax.grid()
            #ax.plot(Angular_Error[1],Angular_Error[0], lw=1, color='blue')
            #fig.savefig('Angular_Error_Graph.png')

            #Steering_Effort[1] = [x - self.start_time for x in Steering_Effort[1]]

            #fig, ax = plt.subplots(figsize=(8, 6), dpi=200)
            #ax.set_xlabel("Time (Seconds)")
            #ax.set_ylabel("Steering Effort")
            #ax.grid()
            #ax.plot(Steering_Effort[1],Steering_Effort[0], lw=1, color='blue')
            #fig.savefig('Steering_Effort_Graph.png')

            rospy.loginfo("Restarting")


if __name__ == '__main__':
    try:
        rospy.init_node('Controller', anonymous=True)
        controller = mainPart()

        rospy.Timer(rospy.Duration(0.01), controller.mainLoop)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass