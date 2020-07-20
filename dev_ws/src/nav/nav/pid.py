import time


class PID:
    """
    Basic Discrete PID Implementation
    """
    def __init__(self, kp=1.0, ki=0.0, kd=1.0, imax=.3, time_ref=None):

        # Constants
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.imax = imax # maximum allowed integration

        # Correction Terms
        self.cp = 0.0
        self.ci = 0.0
        self.cd = 0.0
        self.P = 0.0
        self.I = 0.0
        self.D = 0.0

        self.error_prev = 0.0

        if time_ref is None:
            self.time_ref = time.time()
        else:
            self.time_ref = time_ref

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
        self.time_ref = time_curr
        self.error_prev = error

        self.P = self.kp * self.cp  # Proportional
        self.I = self.ki * self.ci  # Integral
        self.D = self.kd * self.cd  # Derivative
        
        if self.I > self.imax:
            self.I = self.imax
        
        if -1*self.I > self.imax:
            self.I = -1*self.imax
            
        return self.P + self.I + self.D
