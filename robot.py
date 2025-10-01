# robot.py
import math
from dataclasses import dataclass

def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return (a + math.pi) % (2*math.pi) - math.pi

@dataclass
class Robot:
    x: float = 0.0          # meters
    y: float = 0.0          # meters
    theta: float = 0.0      # radians (0 = +x)
    wheel_radius: float = 0.03   # meters
    wheel_base: float = 0.15     # distance between wheels (m)

    def update_from_wheel_angular(self, w_l, w_r, dt):
        """Update robot pose given left/right wheel angular velocities (rad/s)."""
        v_l = self.wheel_radius * w_l
        v_r = self.wheel_radius * w_r
        v = (v_r + v_l) / 2.0
        omega = (v_r - v_l) / self.wheel_base
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta = wrap_angle(self.theta + omega * dt)

    def update_from_v_omega(self, v, omega, dt):
        """Alternative update using body v (m/s) and omega (rad/s)."""
        v_r = v + omega * self.wheel_base / 2.0
        v_l = v - omega * self.wheel_base / 2.0
        w_r = v_r / self.wheel_radius
        w_l = v_l / self.wheel_radius
        self.update_from_wheel_angular(w_l, w_r, dt)
