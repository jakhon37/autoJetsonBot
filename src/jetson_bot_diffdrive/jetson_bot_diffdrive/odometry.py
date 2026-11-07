"""Wheel odometry for diffdrive_node, kept free of ROS so it can be unit tested."""
import math

# Diagonals of /odom's covariances, in the order x, y, z, roll, pitch, yaw. The EKF fuses x, y,
# vx and vyaw. Wheel slip makes the encoders' yaw (0.1) and yaw rate (0.05) the least trusted,
# as imu_nav2_roadmap_v2.md plans: the gyro's rate has 0.01, so the IMU leads on rotation.
# vy is small because the wheels can't slide sideways, and 1e6 marks what a robot on a flat
# floor doesn't measure.
POSE_COVARIANCE_DIAGONAL = [0.001, 0.001, 1e6, 1e6, 1e6, 0.1]
TWIST_COVARIANCE_DIAGONAL = [0.001, 0.001, 1e6, 1e6, 1e6, 0.05]


def covariance(diagonal):
    """Return a row-major 6x6 covariance matrix with the given diagonal."""
    cov = [0.0] * 36
    for i, value in enumerate(diagonal):
        cov[i * 7] = float(value)
    return cov


def parse_encoder_line(line):
    """Return the counts of an 'e <left> <right>' line as (left, right), or None."""
    low = line.lower()
    if not (low.startswith('e ') or low.startswith('e\t')):
        return None
    parts = line.split()
    if len(parts) < 3:
        return None
    try:
        return int(float(parts[1])), int(float(parts[2]))
    except ValueError:
        return None


def encoder_dt(elapsed, n_lines, min_dt):
    """Return the seconds each of a tick's n_lines encoder lines covers.

    Lines that arrive together are handled microseconds apart, so the time between
    handling two of them isn't how far apart the ESP32 sent them. The tick's lines share
    the time since the last tick that had any, and none counts as shorter than min_dt,
    so a late tick followed by an early one can't make a speed spike either.
    """
    return max(elapsed / n_lines, min_dt)


class WheelOdometry:
    """Integrates cumulative wheel counts into a pose and speeds."""

    def __init__(self, wheel_separation, wheel_radius, encoder_cpr):
        self.wheel_sep = wheel_separation
        self.wheel_rad = wheel_radius
        self.encoder_cpr = encoder_cpr
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.left_pos = 0.0
        self.right_pos = 0.0
        self.have_baseline = False

    def reset_baseline(self):
        """Take the next counts as the starting point, e.g. after a reconnect."""
        self.have_baseline = False

    def update(self, l_count, r_count, dt):
        """Integrate one encoder line that covers dt seconds.

        Returns (v_x, v_th, wheel_angles, wheel_rates). The wheel lists are left first,
        in rad and rad/s.
        """
        # Convert counts to meters
        dist_l = (l_count / self.encoder_cpr) * (2 * math.pi * self.wheel_rad)
        dist_r = (r_count / self.encoder_cpr) * (2 * math.pi * self.wheel_rad)

        # The ESP32 keeps its counts while the bridge is down, so start from them.
        if not self.have_baseline:
            self.left_pos = dist_l
            self.right_pos = dist_r
            self.have_baseline = True

        d_l = dist_l - self.left_pos
        d_r = dist_r - self.right_pos
        self.left_pos = dist_l
        self.right_pos = dist_r

        d_dist = (d_l + d_r) / 2.0
        d_th = (d_r - d_l) / self.wheel_sep

        self.x += d_dist * math.cos(self.th + d_th / 2.0)
        self.y += d_dist * math.sin(self.th + d_th / 2.0)
        self.th += d_th

        wheel_angles = [dist_l / self.wheel_rad, dist_r / self.wheel_rad]
        wheel_rates = [d_l / dt / self.wheel_rad, d_r / dt / self.wheel_rad]
        return d_dist / dt, d_th / dt, wheel_angles, wheel_rates
