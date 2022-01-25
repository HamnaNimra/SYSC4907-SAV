import math
import time
import os
from enum import Enum

# The output of the PID controller is always a positive number after being
# translated to a value that AirSim can use for acceleration or braking.
# Thus a distinction needs to be made if the returned value is for accelerating or braking
class ThrottleAction(Enum):
    Accelerate = 0
    Brake = 1

class PIDController:

    def __init__(self):
        # This depends on exp_factor in pidToCarValues()
        self.speedAdjustmentFactor = 1.15
        self.currentSpeed = 0.0
        self.targetSpeed = 5.0 * self.speedAdjustmentFactor

        self.speed_difference_sum = 0
        self.lastError = 0
        self.speedDifference = 0

        pidCoefficientsFile = open(os.path.abspath(os.path.dirname(__file__)) + "/pid_values.txt")

        # Used in integral windup mitigation. See publish_results()
        self.potentialKiValue = float(pidCoefficientsFile.readline())
        self.clampValue = 0.995  # Must be in range of [0, 1)

        self.kP = float(pidCoefficientsFile.readline())
        self.kI = self.potentialKiValue
        self.kD = float(pidCoefficientsFile.readline())

    # Converts the PID controller output value in the range of [0, 1) that the throttle and brake can have
    def pidToCarValues(self, inp):

        # A lower value means a more gradual throttling and acceleration. However a higher input is needed to get a
        # output value that is needed for a car to achieve its target speed than the PID controller will output. To deal
        # with this, make the target speed higher than what is desired internally using self.speedAdjustmentFactor
        exp_factor = -1
        return max(1 - math.exp(exp_factor * inp), 0.0)

    def update_pid_output(self, car_speed, delta_time):

        # Find the inputs to the PID controller: value for proportional, integral and differential

        self.speedDifference = self.targetSpeed - car_speed
        proportional_input = self.speedDifference * self.kP

        self.speed_difference_sum = (self.speed_difference_sum + self.speedDifference) * delta_time
        integral_input = self.speed_difference_sum * self.kI

        differential_input = self.kD * ((self.speedDifference - self.lastError) / delta_time)
        self.lastError = self.speedDifference

        # Compute the value for the throttle or the braking. The output of the PID controller does not correlate to a
        # range of [0, 1] as required for the throttle or brake, hence the call to pidToCarValues

        pid_controller_output = proportional_input + integral_input + differential_input
        translated_value = 0.0
        brake_action = ThrottleAction.Accelerate

        if pid_controller_output > 0.0:
            translated_value = self.pidToCarValues(pid_controller_output)
        else:
            translated_value = self.pidToCarValues(-pid_controller_output)
            brake_action = ThrottleAction.Brake

        # Need to stop integral windup. If PID controller is saying to brake or accelerate but car isn't for whatever reason
        # then the integral will increase in magnitude continously. Then when the car is free to move, the pid output will stay near max
        # throttle or break even as it nears target speed as the integral component will take a long time to reach an appropriate
        # brake or throttle value

        clamping_has_effect = translated_value > self.clampValue
        same_signs = (pid_controller_output > 0.0 and self.speedDifference > 0.0) or (pid_controller_output < 0.0 and self.speedDifference < 0.0)

        # Integral windup occurs when car is told to accelerate when is it below the target speed; hence the same_signs variable.
        # Same logic but in reverse when it comes to speed for braking
        if clamping_has_effect and same_signs:

            # Integral windup may be detected, but if kI is still small let it continue changing. For example, if kI is
            # 0.05 and integral windup is detected but kI is now at 0.25, jumping directly to the potentiaKiValue can
            # result in an abrupt change of kI
            if abs(self.kI) > self.potentialKiValue:
                if pid_controller_output < 0.0:
                    # If car needs to brake and integral windup is being happening, then kI will
                    # become negative. Thus keep it negative when protecting against windup
                    self.kI = -self.potentialKiValue
                else:
                    self.kI = self.potentialKiValue

        return (brake_action, translated_value)
