# controllers.py
class PController:
    def __init__(self, kp, output_limit=None):
        self.kp = kp
        self.output_limit = output_limit

    def update(self, error):
        out = self.kp * error
        if self.output_limit is not None:
            out = max(min(out, self.output_limit), -self.output_limit)
        return out


class PDController:
    def __init__(self, kp, kd, output_limit=None):
        self.kp = kp
        self.kd = kd
        self.prev_error = None
        self.output_limit = output_limit

    def update(self, error, dt):
        if self.prev_error is None:
            de = 0.0
        else:
            de = (error - self.prev_error) / dt
        out = self.kp * error + self.kd * de
        self.prev_error = error
        if self.output_limit is not None:
            out = max(min(out, self.output_limit), -self.output_limit)
        return out


class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None, integrator_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = None
        self.output_limit = output_limit
        self.integrator_limit = integrator_limit

    def update(self, error, dt):
        if self.prev_error is None:
            de = 0.0
        else:
            de = (error - self.prev_error) / dt
        self.integral += error * dt
        if self.integrator_limit is not None:
            self.integral = max(min(self.integral, self.integrator_limit), -self.integrator_limit)
        out = self.kp * error + self.ki * self.integral + self.kd * de
        self.prev_error = error
        if self.output_limit is not None:
            out = max(min(out, self.output_limit), -self.output_limit)
        return out
