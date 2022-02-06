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
        self.speed_adjustment_factor = 1.15
        self.current_speed = 0.0
        self.target_speed = 7.0 * self.speed_adjustment_factor

        self.speed_difference_sum = 0
        self.last_error = 0
        self.speed_difference = 0

        pid_coefficients_file = open(os.path.abspath(os.path.dirname(__file__)) + "/pid_values.txt")

        # Used in integral windup mitigation. See publish_results()
        self.potential_Ki_value = float(pid_coefficients_file.readline())
        self.clampValue = 0.995  # Must be in range of [0, 1)

        self.kP = float(pid_coefficients_file.readline())
        self.kI = self.potential_Ki_value
        self.kD = float(pid_coefficients_file.readline())

    # Converts the PID controller output value in the range of [0, 1) that the throttle and brake can have
    def pidToCarValues(self, inp):

        # A lower value means a more gradual throttling and acceleration. However a higher input is needed to get a
        # output value that is needed for a car to achieve its target speed than the PID controller will output. To deal
        # with this, make the target speed higher than what is desired internally using self.speedAdjustmentFactor
        exp_factor = -1
        return max(1 - math.exp(exp_factor * inp), 0.0)

    def update_pid_output(self, car_speed, delta_time) -> (float, float):

        # Find the inputs to the PID controller: value for proportional, integral and differential

        self.speed_difference = self.target_speed - car_speed
        proportional_input = self.speed_difference * self.kP

        self.speed_difference_sum = (self.speed_difference_sum + self.speed_difference) * delta_time
        integral_input = self.speed_difference_sum * self.kI

        differential_input = self.kD * ((self.speed_difference - self.last_error) / delta_time)
        self.last_error = self.speed_difference

        # Compute the value for the throttle or the braking. The output of the PID controller does not correlate to a
        # range of [0, 1] as required for the throttle or brake, hence the call to pidToCarValues

        pid_controller_output = proportional_input + integral_input + differential_input
        translated_value = 0.0
        action = ThrottleAction.Accelerate

        if pid_controller_output > 0.0:
            translated_value = self.pidToCarValues(pid_controller_output)
        else:
            translated_value = self.pidToCarValues(-pid_controller_output)
            action = ThrottleAction.Brake

        # Need to stop integral windup. If PID controller is saying to brake or accelerate but car isn't for whatever reason
        # then the integral will increase in magnitude continuously. Then, when the car is free to move, the pid output will stay near max
        # throttle or break even as it nears target speed as the integral component will take a long time to reach an appropriate
        # brake or throttle value

        clamping_has_effect = translated_value > self.clampValue
        same_signs = (pid_controller_output > 0.0 and self.speed_difference > 0.0) or (pid_controller_output < 0.0 and self.speed_difference < 0.0)

        # Integral windup occurs when car is told to accelerate when is it below the target speed; hence the same_signs variable.
        # Same logic but in reverse when it comes to speed for braking
        if clamping_has_effect and same_signs:

            # Integral windup may be detected, but if kI is still small let it continue changing. For example, if kI is
            # 0.05 and integral windup is detected but kI is now at 0.25, jumping directly to the potentiaKiValue can
            # result in an abrupt change of kI
            if abs(self.kI) > self.potential_Ki_value:
                if pid_controller_output < 0.0:
                    # If car needs to brake and integral windup is being happening, then kI will
                    # become negative. Thus keep it negative when protecting against windup
                    self.kI = -self.potential_Ki_value
                else:
                    self.kI = self.potential_Ki_value

        return action, translated_value
