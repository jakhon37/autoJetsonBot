#!/usr/bin/env python3
"""Keep the RPLidar A1's motor stopped while nothing else uses the lidar.

The A1's motor runs whenever its USB adapter's DTR line is released, and the CP2102's driver
releases it whenever the port closes, so the lidar spins whenever the stack is down. This holds
the port open with DTR set, as the lidar driver's stop_motor does, until it is killed.
./robot.sh stop and ./robot.sh lidar-off start it in the container. Every launch kills it first,
so the lidar driver gets the port to itself and starts the motor again.
"""
import os
import sys
import time

import serial

sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                                'src', 'jetson_bot_diffdrive'))
from jetson_bot_diffdrive.serial_ports import find_lidar_port  # noqa: E402


def holders(port):
    """Return the command lines of other processes that have the port open."""
    real = os.path.realpath(port)
    found = []
    for pid in filter(str.isdigit, os.listdir('/proc')):
        if int(pid) == os.getpid():
            continue
        try:
            fds = os.listdir(f'/proc/{pid}/fd')
        except OSError:
            continue
        for fd in fds:
            try:
                if os.path.realpath(f'/proc/{pid}/fd/{fd}') != real:
                    continue
                with open(f'/proc/{pid}/cmdline', 'rb') as f:
                    found.append(f.read().replace(b'\0', b' ').decode(errors='replace').strip())
            except OSError:
                continue
            break
    return found


def main():
    port, name = find_lidar_port()
    if not port:
        print('Lidar motor: no lidar on USB')
        return 0
    others = holders(port)
    if others:
        print(f'Lidar motor: left as it is, {port} is in use by {others[0][:80]}')
        return 0
    lidar = serial.Serial()
    lidar.port = port
    lidar.baudrate = 115200
    lidar.dtr = True  # set: the motor stops, as with the lidar driver's stop_motor
    lidar.rts = False
    lidar.open()
    print(f'Lidar motor: stopped, holding {port} ({name}) until this is killed', flush=True)
    while True:
        time.sleep(3600)


if __name__ == '__main__':
    sys.exit(main())
