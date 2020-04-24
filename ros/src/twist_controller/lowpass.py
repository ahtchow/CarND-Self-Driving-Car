
class LowPassFilter(object):
    def __init__(self, lpf_constants):
        self.tau = lpf_constants[0]
        self.ts = lpf_constants[1]
        self.a = 1. / (self.tau / self.ts + 1.)
        self.b = self.tau / self.ts / (self.tau / self.ts + 1.);

        self.last_val = 0.
        self.ready = False

    def get(self):
        return self.last_val

    def filt(self, val):
        if self.ready:
            val = self.a * val + self.b * self.last_val
        else:
            self.ready = True

        self.last_val = val
        return val
