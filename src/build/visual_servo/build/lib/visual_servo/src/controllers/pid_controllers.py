# Copied from marker repo

class pid():
    def __init__(self, kp, ki=0, kd=0, integral_limit=0.2,
                 b0=15.232, b1=-14.730, a1=-0.498, gain=1.005, parent=None):
        self.parent = parent
        if kp is None:
            kp = 0
        if ki is None:
            ki = 0
        if kd is None:
            kd = 0
        # Save the controller parametes

        self.kd = kd

        self.prev_error = 0
        self.integral = 0

        # PD
        self.d1 = None
        self.b0 = b0
        self.b1 = b1
        self.a1 = a1

        # PI
        self.gain = gain
        self.kp = kp
        self.ki = ki
        # Limits
        self.integral_limit = integral_limit

        # unused. Just for printint / logging
        self.error = None

    def update_simple(self, error):
        self.integral += error
        derivative = error - self.prev_error

        self.prev_error = error

        p = self.kp * error
        i = self.ki * self.integral
        d = self.kd * derivative

        # self.integrator += self.ki * (error + self.prev_error) * t
        return p + i + d

    def update_pd(self, feedback):
        # Something might be wrong with d1
        if self.d1 is None:
            self.d1 = (self.b1 - self.b0 * self.a1) / (1 + self.a1) * feedback
        output = self.b0 * feedback + self.d1
        self.d1 = self.b1 * feedback - self.a1 * output
        # Used for tuning the controller
        # self.parent.camera.x = output
        # self.parent.camera.y = output
        # self.parent.camera.z = output
        return output

    def update_pi(self, reference, feedback):
        error = (reference - feedback) * self.kp
        self.error = error
        output = self.integral + error * self.gain
        self.integral += self.ki * error

        self.integral = self.limit(self.integral, self.integral_limit)
        # Used for tuning the controller
        # self.parent.camera.x_raw = output
        # self.parent.camera.y_raw = output
        # self.parent.camera.z_raw = output
        return output

    def update(self, reference, feedback):
        pd_output = self.update_pd(feedback)
        pi_output = self.update_pi(reference, pd_output)
        return pi_output

    def limit(self, input, limit):
        if input > limit:
            input = limit
        elif input < -limit:
            input = -limit
        return input

    def reset_controller(self):
        self.integral = 0
        self.d1 = None
