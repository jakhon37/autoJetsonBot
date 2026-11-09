"""Unit tests for the bridge's odometry. odometry.py imports no ROS, so they run anywhere."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jetson_bot_diffdrive.odometry import (  # noqa: E402
    POSE_COVARIANCE_DIAGONAL, TWIST_COVARIANCE_DIAGONAL, WheelOdometry, covariance,
    encoder_dt, parse_encoder_line)

SEP, RADIUS, CPR = 0.212, 0.034, 1497
PERIOD = 0.02  # the ESP32 sends an 'e' line every 20 ms
V, W = 0.2, 0.5  # speed of the simulated robot, m/s and rad/s
# Counts are whole numbers, so one line's speed can be off by one count per wheel: about
# 7 mm/s at 1,497 counts per turn and 50 Hz. A bad dt gives spikes many times larger.
COUNT_V = 2 * math.pi * RADIUS / CPR / PERIOD
COUNT_W = 2 * COUNT_V / SEP


def _counts(t):
    """Return the cumulative (left, right) counts after driving at V and W for t seconds."""
    per_m = CPR / (2 * math.pi * RADIUS)
    return round((V - W * SEP / 2) * t * per_m), round((V + W * SEP / 2) * t * per_m)


def test_parse_encoder_line():
    """Read the counts from 'e' lines and ignore everything else."""
    assert parse_encoder_line('e 120 -45') == (120, -45)
    assert parse_encoder_line('E\t12.0 7') == (12, 7)
    assert parse_encoder_line('e 120') is None
    assert parse_encoder_line('e left right') is None
    assert parse_encoder_line('i 0.1 0.2 9.8 0 0 0') is None
    assert parse_encoder_line('READY') is None


def test_encoder_dt():
    """Share a tick's time between its lines, never below one period."""
    assert encoder_dt(0.020, 1, PERIOD) == PERIOD  # one line per tick
    assert encoder_dt(0.040, 2, PERIOD) == PERIOD  # two lines read together
    assert encoder_dt(0.002, 1, PERIOD) == PERIOD  # an early tick right after a late one
    assert encoder_dt(0.220, 11, PERIOD) == PERIOD  # lines queued during a stall
    assert encoder_dt(0.040, 1, PERIOD) == 0.040  # after a lost line, this one covers two
    assert encoder_dt(0.050, 1, PERIOD) == 0.050  # a 20 Hz firmware


def test_speed_does_not_spike_when_lines_bunch_up():
    """Feed a 50 Hz ESP32 to a jittery 50 Hz timer, the way update_loop does."""
    arrivals = {i: i * PERIOD + 0.0003 for i in range(60)}
    del arrivals[50]  # lost on the way
    ticks = [k * PERIOD + 0.001 for k in range(60)]
    ticks[10] = 10 * PERIOD + 0.0001  # early, so line 10 waits for tick 11
    ticks[20] = 20 * PERIOD + 0.019  # late, and tick 21 follows 2 ms after it
    ticks = [t for k, t in enumerate(ticks) if not 30 <= k < 40]  # a 200 ms stall

    odo = WheelOdometry(SEP, RADIUS, CPR)
    pending = sorted(arrivals)
    last_tick = 0.0
    speeds = {}
    for tick in ticks:
        batch = [i for i in pending if arrivals[i] <= tick]
        pending = [i for i in pending if arrivals[i] > tick]
        if not batch:
            continue
        dt = encoder_dt(tick - last_tick, len(batch), PERIOD)
        last_tick = tick
        for i in batch:
            v, w, _, _ = odo.update(*_counts(i * PERIOD), dt)
            speeds[i] = (v, w)
    del speeds[0]  # the first line only sets the baseline

    assert max(v for v, _ in speeds.values()) < V * 1.05 + COUNT_V
    assert max(w for _, w in speeds.values()) < W * 1.05 + COUNT_W
    # Read together, right after a late tick, after the stall, after the lost line.
    for i in (10, 11, 21, 30, 35, 40, 51):
        v, w = speeds[i]
        assert abs(v - V) < V * 0.05 + COUNT_V, (i, v)
        assert abs(w - W) < W * 0.05 + COUNT_W, (i, w)

    # The pose comes from the counts alone, so dt can't move it.
    t = 59 * PERIOD
    assert abs(odo.th - W * t) < 0.002
    assert abs(odo.x - V / W * math.sin(W * t)) < 0.002
    assert abs(odo.y - V / W * (1 - math.cos(W * t))) < 0.002


def test_counts_after_a_reset_only_set_the_baseline():
    """Start from the ESP32's counts after a reconnect or a reboot, without a jump."""
    odo = WheelOdometry(SEP, RADIUS, CPR)
    v, w, _, _ = odo.update(50000, -20000, PERIOD)
    assert (v, w, odo.x, odo.y, odo.th) == (0.0, 0.0, 0.0, 0.0, 0.0)
    odo.update(50100, -19900, PERIOD)
    x = odo.x
    assert x > 0.0 and abs(odo.th) < 1e-12  # both wheels moved 100 counts
    odo.reset_baseline()
    v, w, _, _ = odo.update(0, 0, PERIOD)  # rebooted, counting from zero again
    assert (v, w, odo.x) == (0.0, 0.0, x)


def test_covariance_fills_the_diagonal():
    """Build a row-major 6x6 matrix with nothing off the diagonal."""
    cov = covariance([1.0, 2.0, 3.0, 4.0, 5.0, 6.0])
    assert len(cov) == 36
    assert [cov[i * 7] for i in range(6)] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
    assert sum(cov) == 21.0


def test_odom_covariance_is_not_zero():
    """Give the EKF real variances, trusting the gyro over the encoders for rotation."""
    for diagonal in (POSE_COVARIANCE_DIAGONAL, TWIST_COVARIANCE_DIAGONAL):
        assert len(diagonal) == 6
        assert all(isinstance(v, float) and v > 0.0 for v in diagonal)
    # imu_bridge_node.py gives the gyro's rates a variance of 0.01.
    assert TWIST_COVARIANCE_DIAGONAL[5] > 0.01


if __name__ == '__main__':
    test_parse_encoder_line()
    test_encoder_dt()
    test_speed_does_not_spike_when_lines_bunch_up()
    test_counts_after_a_reset_only_set_the_baseline()
    test_covariance_fills_the_diagonal()
    test_odom_covariance_is_not_zero()
