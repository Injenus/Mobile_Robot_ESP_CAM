class PID():
    def __init__(self, kp, ki, kd, dt, min_v=11, max_v=99):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.dt = dt
        self.min_v = min_v
        self.max_v = max_v
        self.prev_err = 0
        self.err = 0
        self.res = self.min_v
        self.P = 0
        self.I = 0
        self.D = 0

    def calc(self, inp, setp=0):
        self.P = inp - setp
        self.I = self.I + self.P * self.dt
        if self.I < self.min_v:
            self.I = self.min_v
        if self.I > self.max_v:
            self.I = self.max_v
        self.D = (self.P - self.prev_err)/self.dt
        self.prev_err = self.P
        self.res = int(self.P*self.kp + self.I*self.ki + self.D*self.kd + 0.5)
        if self.res < self.min_v:
            self.res = self.min_v
        if self.res > self.max_v:
            self.res = self.max_v
        return self.res
