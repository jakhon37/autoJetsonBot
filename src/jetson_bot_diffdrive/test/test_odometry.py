"""Unit tests for the bridge's odometry. odometry.py imports no ROS, so they run anywhere."""
import math
import os
import random
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from jetson_bot_diffdrive.odometry import (  # noqa: E402
    POSE_COVARIANCE_DIAGONAL, STAMP_WRAP, TWIST_COVARIANCE_DIAGONAL, WheelOdometry, covariance,
    encoder_dt, parse_encoder_line)

SEP, RADIUS, CPR = 0.212, 0.034, 1497
PERIOD = 0.02  # the ESP32 sends an 'e' line every 20 ms
CONTROL_PERIOD = 0.01  # its control task reads the counters every 10 ms
V, W = 0.2, 0.5  # speed of the simulated robot, m/s and rad/s
# Counts are whole numbers, so one line's speed can be off by one count per wheel: about
# 7 mm/s at 1,497 counts per turn and 50 Hz. A bad dt gives spikes many times larger.
COUNT_V = 2 * math.pi * RADIUS / CPR / PERIOD
COUNT_W = 2 * COUNT_V / SEP
# The ESP32's clock when the simulated drives start, in microseconds: 0.6 s before its
# 32-bit clock wraps, so every run below crosses the wrap.
CLOCK_AT_START = STAMP_WRAP - 600_000


def _counts(t):
    """Return the cumulative (left, right) counts after driving at V and W for t seconds."""
    per_m = CPR / (2 * math.pi * RADIUS)
    return round((V - W * SEP / 2) * t * per_m), round((V + W * SEP / 2) * t * per_m)


def _run(arrivals, ticks, stamped=True, read_at=None):
    """Feed 'e' lines to the odometry on the bridge's 50 Hz timer, the way update_loop does.

    arrivals maps each line's number to when it reaches the Jetson, and ticks are when the
    timer reads the port. Line i holds the counts the ESP32 read at read_at[i] (i periods
    by default) and, if stamped, that time. Returns {i: (v, w, tick that handled it)} and
    the odometry.
    """
    read_at = read_at or {i: i * PERIOD for i in arrivals}
    odo = WheelOdometry(SEP, RADIUS, CPR)
    lines = sorted(arrivals)  # a serial port keeps them in order
    last_stamp = None
    speeds = {}
    for tick in ticks:
        while lines and arrivals[lines[0]] <= tick:
            i = lines.pop(0)
            stamp = (CLOCK_AT_START + round(read_at[i] * 1e6)) % STAMP_WRAP if stamped else None
            dt = encoder_dt(stamp, last_stamp, PERIOD)
            last_stamp = stamp
            v, w, _, _ = odo.update(*_counts(read_at[i]), dt)
            speeds[i] = (v, w, tick)
    del speeds[min(speeds)]  # the first line only sets the baseline
    return speeds, odo


def _bunched_lines():
    """A 50 Hz ESP32 and a jittery 50 Hz timer: lines read together, late, after a stall, lost."""
    arrivals = {i: i * PERIOD + 0.0003 for i in range(60)}
    del arrivals[50]  # lost on the way
    ticks = [k * PERIOD + 0.001 for k in range(60)]
    ticks[10] = 10 * PERIOD + 0.0001  # early, so line 10 waits for tick 11
    ticks[20] = 20 * PERIOD + 0.019  # late, and tick 21 follows 2 ms after it
    ticks = [t for k, t in enumerate(ticks) if not 30 <= k < 40]  # a 200 ms stall
    return arrivals, ticks


def test_parse_encoder_line():
    """Read the counts, and the ESP32's time if the line has it, and ignore everything else."""
    assert parse_encoder_line('e 120 -45') == (120, -45, None)
    assert parse_encoder_line('E\t12.0 7') == (12, 7, None)
    assert parse_encoder_line('e 120 -45 4294967295') == (120, -45, 4294967295)
    assert parse_encoder_line('e 120') is None
    assert parse_encoder_line('e left right') is None
    assert parse_encoder_line('e 120 -45 soon') is None
    assert parse_encoder_line('i 0.1 0.2 9.8 0 0 0') is None
    assert parse_encoder_line('READY') is None


def test_encoder_dt():
    """Time a line by the ESP32's stamps, or by one period when it has none."""
    assert encoder_dt(None, None, PERIOD) == PERIOD  # firmware that sends no time
    assert encoder_dt(None, 1_000_000, PERIOD) == PERIOD
    assert encoder_dt(1_000_000, None, PERIOD) == PERIOD  # the first stamped line
    assert encoder_dt(1_020_000, 1_000_000, PERIOD) == 0.020
    assert encoder_dt(1_040_000, 1_000_000, PERIOD) == 0.040  # after a lost line
    assert encoder_dt(1_030_000, 1_000_000, PERIOD) == 0.030  # counts from a later 10 ms read
    assert encoder_dt(15_000, STAMP_WRAP - 5_000, PERIOD) == 0.020  # across the clock's wrap
    assert encoder_dt(1_000_000, 1_000_000, PERIOD) is None  # the same counts again


def test_speed_does_not_spike_when_lines_bunch_up():
    """Give every line its own speed, however the timer reads them, with the ESP32's stamps."""
    speeds, odo = _run(*_bunched_lines())

    assert max(v for v, _, _ in speeds.values()) < V * 1.05 + COUNT_V
    assert max(w for _, w, _ in speeds.values()) < W * 1.05 + COUNT_W
    # Read together, right after a late tick, after the stall, after the lost line.
    for i in (10, 11, 21, 30, 35, 40, 51):
        v, w, _ = speeds[i]
        assert abs(v - V) < V * 0.05 + COUNT_V, (i, v)
        assert abs(w - W) < W * 0.05 + COUNT_W, (i, w)

    # The pose comes from the counts alone, so dt can't move it.
    t = 59 * PERIOD
    assert abs(odo.th - W * t) < 0.002
    assert abs(odo.x - V / W * math.sin(W * t)) < 0.002
    assert abs(odo.y - V / W * (1 - math.cos(W * t))) < 0.002


def test_lines_without_a_stamp_count_one_period_each():
    """Without stamps, the line after a lost one reads double, and nothing else moves."""
    speeds, odo = _run(*_bunched_lines(), stamped=False)

    for i, (v, w, _) in speeds.items():
        expected = 2 if i == 51 else 1  # line 51 holds the counts of two periods
        assert abs(v - expected * V) < V * 0.05 + COUNT_V, (i, v)
        assert abs(w - expected * W) < W * 0.05 + COUNT_W, (i, w)
    t = 59 * PERIOD
    assert abs(odo.th - W * t) < 0.002


def test_mean_speed_is_unbiased_when_the_timer_runs_late():
    """On a busy Jetson the speeds average out to the wheels' speed, stamped or not.

    A minute of lines, with a third of the timer's ticks late by up to 15 ms and 3% of
    them after a stall of 20 to 120 ms. Timing each line by its tick, at least 1/50 s,
    read 7% low here. Each line holds the counts of the control task's last 10 ms read,
    and a fifth of them the read before that, as when that read and the sending fall in
    the same millisecond. Without stamps those lines read half or one and a half times
    the speed, which averages out but moves the EKF's distance by about 1% a minute.
    """
    rng = random.Random(24)
    n = 3000
    read_at = {i: i * PERIOD - (CONTROL_PERIOD if rng.random() < 0.2 else 0.0)
               for i in range(n)}
    arrivals = {i: i * PERIOD + rng.uniform(0.0002, 0.001) for i in range(n)}
    ticks = []
    for k in range(n + 10):
        late = rng.uniform(0.0, 0.015) if rng.random() < 0.33 else 0.0
        if rng.random() < 0.03:
            late = rng.uniform(0.02, 0.12)
        tick = k * PERIOD + 0.001 + late
        if not ticks or tick > ticks[-1]:  # ticks due during a stall collapse into it
            ticks.append(tick)

    for stamped in (True, False):
        speeds, _ = _run(arrivals, ticks, stamped, read_at)
        lines = sorted(speeds)
        mean_v = sum(speeds[i][0] for i in lines) / len(lines)
        mean_w = sum(speeds[i][1] for i in lines) / len(lines)
        assert abs(mean_v / V - 1) < 0.005, (stamped, mean_v)
        assert abs(mean_w / W - 1) < 0.005, (stamped, mean_w)

        if stamped:
            # The EKF holds each speed until the next line is handled, and adds that up.
            handled = [speeds[i][2] for i in lines]
            ekf = sum(speeds[i][0] * (t_next - t)
                      for i, t, t_next in zip(lines, handled, handled[1:]))
            assert abs(ekf / (V * (handled[-1] - handled[0])) - 1) < 0.005, ekf


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
    test_lines_without_a_stamp_count_one_period_each()
    test_mean_speed_is_unbiased_when_the_timer_runs_late()
    test_counts_after_a_reset_only_set_the_baseline()
    test_covariance_fills_the_diagonal()
    test_odom_covariance_is_not_zero()
