from pid import PID
from lowpass import LowPassFilter
from yaw_controller import YawController
import rospy

# GAS_DENSITY = 2.858
# ONE_MPH = 0.44704

class Controller(object):
    
    def __init__(self, param):

        # Store Parameters
        self.param = param
        self.param['min_speed'] = 0.1
        # Yaw Controller
        self.yaw_controller = YawController(param)

        # PID Controller Constants - [kp, ki, kd, min_throttle, max_throttle]
        self.pid_constants = (0.3, 0.1 , 0. , 0., 0.2)
        
        # Pid Controller
        self.throttle_controller = PID(self.pid_constants)

        # Low Pass Fitler Constants - [tau, ts]
        self.lpf_constants = (0.5, 0.02)

        # Low Pass Filter - Filters out all the high-frequency noise in velocity
        self.vel_lpf = LowPassFilter(self.lpf_constants)

        self.last_vel = 0.0
        self.last_time = rospy.get_time()

    def control(self, current_vel, dbw_enabled, target_vel, angular_vel):

        # Check if DBW is enabled
        if not dbw_enabled:
            self.throttle_controller.reset()
            return 0., 0., 0.

        # Fitler current_vel
        current_vel = self.vel_lpf.filt(current_vel)
        vel_error = target_vel - current_vel
        self.last_vel = current_vel

        # Use Yaw Controller
        steering_angle = self.yaw_controller.get_steering(target_vel, angular_vel, current_vel)

        # Update Clocking
        current_time = rospy.get_time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # PID Controller 
        throttle = self.throttle_controller.step(vel_error, delta_time)
        brake = 0

        # Linear Velocity is 0 and going very slow, probably try to stop
        if target_vel == 0. and current_vel < 0.1:
            throttle = 0
            brake = 700
        
        # Velocity Error is negative so we are going too fast, slow down.
        elif throttle < .1 and vel_error < 0:
            throttle = 0
            decel = max(vel_error, self.param['decel_limit'])
            brake = abs(decel) * self.param['vehicle_mass'] * self.param['wheel_radius'] # Torque (N*m)

        return throttle, brake, steering_angle



