import time


class PID:
    """
    Basic Discrete PID Implementation
    """
    def __init__(self, kp=1.0, ki=0.0, kd=1.0, time_ref=None):

        # Constants
        self.kp = kp
        self.ki = ki
        self.kd = kd

        # Correction Terms
        self.cp = 0.0
        self.ci = 0.0
        self.cd = 0.0

        self.error_prev = 0.0

        if time_prev is None:
            self.time_prev = time.time()
        else:
            self.time_prev = time_prev

    def update(self, error, time_curr=None):

        if time_curr is None:
            time_curr = time.time()

        dt = time_curr - self.time_ref
        if dt <= 0:
            return 0

        de = error - self.error_prev

        self.cp = error
        self.ci += error*dt
        self.cd = de/dt

        # update values
        self.time_prev = time_curr
        self.error_prev = error

        return (
            (self.kp * self.cp)  # Proportional
            + (self.ki * self.ci)  # Integral
            + (self.kd * self.cd)  # Derivative
        )

        # TODO: plot recorded results