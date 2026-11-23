"""Tests for the motor board's and the lidar's port searches. serial_ports.py imports no ROS, so
they run anywhere.

Each test makes up a /dev and a /sys/class/tty, the way the Jetson host, the robot's container
(which has no /dev/serial/by-id) or /dev/serial/by-id alone show the devices.
"""
import os
import sys
import tempfile

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)

from jetson_bot_diffdrive.serial_ports import (  # noqa: E402
    com_socket_warning, find_esp32_port, find_lidar_port)

REPO_ROOT = os.path.dirname(os.path.dirname(PKG_DIR))

# (tty, /dev/serial/by-id name, USB vendor, product ID, product name), with made-up serials
NATIVE = ('ttyACM1', 'usb-Espressif_USB_JTAG_serial_debug_unit_00:00:00:00:00:01-if00',
          '303a', '1001', 'USB JTAG/serial debug unit')
COM = ('ttyACM0', 'usb-1a86_USB_Single_Serial_0000000001-if00',
       '1a86', '55d3', 'USB Single Serial')
LIDAR = ('ttyUSB0', 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
         '10c4', 'ea60', 'CP2102 USB to UART Bridge Controller')
CH340 = ('ttyUSB1', 'usb-1a86_USB_Serial-if00-port0', '1a86', '7523', 'USB Serial')

# What a search sees: (/dev/serial/by-id, sysfs). The robot's container has no /dev/serial/by-id.
HOST, BY_ID_ONLY, CONTAINER = (True, True), (True, False), (False, True)


def _find_esp32_port(devices, view):
    """Run find_esp32_port on a made-up /dev and /sys/class/tty holding the devices."""
    (port, name, socket), dev = _search(devices, view, find_esp32_port)
    return port and os.path.relpath(port, dev), name, socket


def _find_lidar_port(devices, view):
    """Run find_lidar_port on a made-up /dev and /sys/class/tty holding the devices."""
    (port, name), dev = _search(devices, view, find_lidar_port)
    return port and os.path.relpath(port, dev), name


def _search(devices, view, search):
    """Make up a /dev and /sys/class/tty holding the devices, and run a search on them."""
    by_id, sysfs = view
    with tempfile.TemporaryDirectory() as root:
        root = os.path.realpath(root)
        dev = os.path.join(root, 'dev')
        sys_class_tty = os.path.join(root, 'sys', 'class', 'tty')
        by_id_dir = os.path.join(dev, 'serial', 'by-id')
        os.makedirs(by_id_dir if by_id else dev)
        for number, (tty, name, vendor, product_id, product) in enumerate(devices, 1):
            open(os.path.join(dev, tty), 'w').close()  # stands in for the device node
            if by_id:
                os.symlink(os.path.join('..', '..', tty), os.path.join(by_id_dir, name))
            if sysfs:
                usb = os.path.join(root, 'sys', 'devices', 'usb1', f'1-2.{number}')
                interface = os.path.join(usb, f'1-2.{number}:1.0')
                # cdc_acm's tty belongs to the USB interface, a usb-serial tty to a port below it
                device = interface if tty.startswith('ttyACM') else os.path.join(interface, tty)
                os.makedirs(device)
                for attr, value in zip(('idVendor', 'idProduct', 'product'),
                                       (vendor, product_id, product)):
                    with open(os.path.join(usb, attr), 'w') as f:
                        f.write(value + '\n')
                os.makedirs(os.path.join(sys_class_tty, tty))
                os.symlink(device, os.path.join(sys_class_tty, tty, 'device'))
        return search(dev, sys_class_tty), dev


def test_esp32_native_usb_comes_before_its_com_socket():
    """Take the ESP32-S3's native USB when its COM socket is plugged in too."""
    # /dev/serial/by-id lists the CH343 (1a86) before Espressif, and the old searches took it.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        port, _, socket = _find_esp32_port([COM, NATIVE, LIDAR], view)
        assert (port, socket) == ('ttyACM1', 'native'), view


def test_esp32_com_socket_alone_is_flagged():
    """Mark the COM socket, where the motor firmware sends nothing."""
    # From 2026-11-11 to 11-23 the cable sat in the COM socket, and launch took its /dev/ttyACM0
    # without a word, so the 2026-11-17 run got no wheel or IMU data.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        port, _, socket = _find_esp32_port([COM, LIDAR], view)
        assert (port, socket) == ('ttyACM0', 'com'), view


def test_esp32_port_is_named():
    """Name the device by its by-id name, or by its USB ID without one."""
    assert _find_esp32_port([NATIVE], HOST)[1] == NATIVE[1]
    assert _find_esp32_port([NATIVE], CONTAINER)[1] == '303a:1001 USB JTAG/serial debug unit'
    assert _find_esp32_port([COM], CONTAINER)[1] == '1a86:55d3 USB Single Serial'


def test_unknown_ttyacm_is_still_taken():
    """Still take a ttyACM that can't be identified, as before."""
    unknown = ('ttyACM0', 'usb-Unknown_Serial-if00', '', '', '')
    port, _, socket = _find_esp32_port([unknown], (False, False))
    assert (port, socket) == ('ttyACM0', '')


def test_ttyusb_is_never_the_motor_board():
    """Never take the lidar's CP2102 or a CH340 for the motor board."""
    # Both sockets of the ESP32-S3 are ttyACM. A CH340 on ttyUSB, like the ESP32-CAM's, shares
    # the CH343's vendor ID, and opening its port resets that board.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        assert _find_esp32_port([LIDAR, CH340], view) == (None, None, ''), view
    assert _find_esp32_port([], HOST) == (None, None, '')


def test_lidar_is_its_cp2102_not_the_first_ttyusb():
    """Find the lidar's CP2102 by name or USB ID, also behind a CH340 on the first ttyUSB."""
    # robot.sh holds this port's DTR line to stop the lidar's motor, and held on a CH340 like
    # the ESP32-CAM's programmer, DTR keeps that board in reset.
    ch340_first = ('ttyUSB0',) + CH340[1:]
    lidar_second = ('ttyUSB1',) + LIDAR[1:]
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        assert _find_lidar_port([ch340_first, lidar_second, NATIVE], view)[0] == 'ttyUSB1', view
    assert _find_lidar_port([LIDAR], HOST)[1] == LIDAR[1]
    container_name = _find_lidar_port([LIDAR], CONTAINER)[1]
    assert container_name == '10c4:ea60 CP2102 USB to UART Bridge Controller'


def test_no_lidar_means_none():
    """Take nothing when no CP2102 is there, not a CH340, the motor board or an unknown ttyUSB."""
    unknown = ('ttyUSB0', 'usb-Unknown_Serial-if00-port0', '', '', '')
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        assert _find_lidar_port([CH340, NATIVE, COM], view) == (None, None), view
    assert _find_lidar_port([unknown], (False, False)) == (None, None)
    assert _find_lidar_port([], HOST) == (None, None)


def test_launch_and_the_lidar_motor_stop_share_the_lidar_rule():
    """Make launch and tools/lidar_motor_off.py take the lidar's port from serial_ports.py."""
    launch = os.path.join('src', 'jetson_bot_bringup', 'launch', 'main.launch.py')
    for path in (launch, os.path.join('tools', 'lidar_motor_off.py')):
        with open(os.path.join(REPO_ROOT, path), 'r') as f:
            text = f.read()
        assert 'find_lidar_port' in text and "'silicon_labs'" not in text, path


def test_com_socket_warning_says_what_to_do():
    """Name the port in the warning, and say where the cable goes."""
    message = com_socket_warning('/dev/ttyACM0', '1a86:55d3 USB Single Serial')
    assert '/dev/ttyACM0' in message and '1a86:55d3' in message
    assert 'COM socket' in message and 'native USB socket' in message


def test_launch_bridge_and_status_share_these_rules():
    """Make launch, the bridge and ./robot.sh status take the port from serial_ports.py."""
    # Each had its own list of names, and they drifted apart: all three took the COM socket's
    # CH343 as readily as the native USB, and robot.sh status had no name for Espressif's.
    for path in (os.path.join('src', 'jetson_bot_bringup', 'launch', 'main.launch.py'),
                 os.path.join('src', 'jetson_bot_diffdrive', 'jetson_bot_diffdrive',
                              'diffdrive_node.py'),
                 'robot.sh'):
        with open(os.path.join(REPO_ROOT, path), 'r') as f:
            text = f.read()
        assert 'jetson_bot_diffdrive.serial_ports import' in text, path
        assert 'qinheng' not in text and 'usb_single_serial' not in text, path


if __name__ == '__main__':
    test_esp32_native_usb_comes_before_its_com_socket()
    test_esp32_com_socket_alone_is_flagged()
    test_esp32_port_is_named()
    test_unknown_ttyacm_is_still_taken()
    test_ttyusb_is_never_the_motor_board()
    test_lidar_is_its_cp2102_not_the_first_ttyusb()
    test_no_lidar_means_none()
    test_launch_and_the_lidar_motor_stop_share_the_lidar_rule()
    test_com_socket_warning_says_what_to_do()
    test_launch_bridge_and_status_share_these_rules()
