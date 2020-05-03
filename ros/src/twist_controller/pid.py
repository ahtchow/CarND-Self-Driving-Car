
MIN_NUM = float('-inf')
MAX_NUM = float('inf')


class PID(object):
    def __init__(self, pid_constants):
        self.kp = pid_constants[0]
        self.ki = pid_constants[1]
        self.kd = pid_constants[2]
        self.min = pid_constants[3]
        self.max = pid_constants[4]

        self.int_val = self.last_error = 0.

    def reset(self):
        self.int_val = 0.0

    def step(self, error, sample_time):

        integral = self.int_val + error * sample_time
        derivative = (error - self.last_error) / sample_time

        val = self.kp * error + self.ki * integral + self.kd * derivative

        if val > self.max:
            val = self.max
        elif val < self.min:
            val = self.min
        else:
            self.int_val = integral
        self.last_error = error

        return val
