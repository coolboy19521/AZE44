from time import monotonic

class PID:
    def __init__(self, coeff):
        self.coeff = coeff
        self.sum_error = 0
        self.last_error = 0
        self.last_time = None

    def __call__(self, x):
        now = monotonic()

        if self.last_time is None:
            self.last_time = now
            dt = 5e-2
        else:
            dt = now - self.last_time
            self.last_time = now

        self.sum_error += x * dt
        y = x * self.coeff[0] + self.sum_error * self.coeff[1] + \
        (x - self.last_error) / dt * self.coeff[2]
        self.last_error = x
        return y
