"""Tests for the motor board's port search. serial_ports.py imports no ROS, so they run anywhere.

Each test makes up a /dev and a /sys/class/tty, the way the Jetson host, the robot's container
(which has no /dev/serial/by-id) or /dev/serial/by-id alone show the devices.
"""
import os
import sys
import tempfile

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG_DIR)

from jetson_bot_diffdrive.serial_ports import com_socket_warning, find_esp32_port  # noqa: E402

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
        port, name, socket = find_esp32_port(dev, sys_class_tty)
        return port and os.path.relpath(port, dev), name, socket


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
    test_com_socket_warning_says_what_to_do()
    test_launch_bridge_and_status_share_these_rules()
