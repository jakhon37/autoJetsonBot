# Copyright 2026 Mac37
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import ast
import os
import tempfile

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LAUNCH_FILE = os.path.join(PKG_DIR, 'launch', 'main.launch.py')

# (tty, /dev/serial/by-id name, USB vendor, product ID, product name), with made-up serials
NATIVE = ('ttyACM1', 'usb-Espressif_USB_JTAG_serial_debug_unit_00:00:00:00:00:01-if00',
          '303a', '1001', 'USB JTAG/serial debug unit')
COM = ('ttyACM0', 'usb-1a86_USB_Single_Serial_0000000001-if00',
       '1a86', '55d3', 'USB Single Serial')
LIDAR = ('ttyUSB0', 'usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0',
         '10c4', 'ea60', 'CP2102 USB to UART Bridge Controller')
CH340 = ('ttyUSB1', 'usb-1a86_USB_Serial-if00-port0', '1a86', '7523', 'USB Serial')

# What launch sees: (/dev/serial/by-id, sysfs). The robot's container has no /dev/serial/by-id.
HOST, BY_ID_ONLY, CONTAINER = (True, True), (True, False), (False, True)


def _launch_functions():
    """Return main.launch.py's functions, run without the launch packages it imports."""
    with open(LAUNCH_FILE, 'r') as f:
        tree = ast.parse(f.read(), LAUNCH_FILE)
    functions = [node for node in tree.body if isinstance(node, ast.FunctionDef)]
    namespace = {'os': os}
    exec(compile(ast.Module(body=functions, type_ignores=[]), LAUNCH_FILE, 'exec'), namespace)
    return namespace


LAUNCH = _launch_functions()


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
        port, name, socket = LAUNCH['find_esp32_port'](dev, sys_class_tty)
        return port and os.path.relpath(port, dev), name, socket


def test_esp32_native_usb_comes_before_its_com_socket():
    """Verify that launch takes the ESP32-S3's native USB when its COM socket is plugged in too."""
    # /dev/serial/by-id lists the CH343 (1a86) before Espressif, and the old launch took it.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        port, _, socket = _find_esp32_port([COM, NATIVE, LIDAR], view)
        assert (port, socket) == ('ttyACM1', 'native'), view


def test_esp32_com_socket_alone_is_flagged():
    """Verify that launch marks the COM socket, where the motor firmware sends nothing."""
    # From 2026-11-11 to 11-23 the cable sat in the COM socket, and launch took its /dev/ttyACM0
    # without a word, so the 2026-11-17 run got no wheel or IMU data. Its audit now warns.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        port, _, socket = _find_esp32_port([COM, LIDAR], view)
        assert (port, socket) == ('ttyACM0', 'com'), view


def test_esp32_port_is_named_for_the_audit():
    """Verify that the audit names the device by its by-id name, or by its USB ID without one."""
    assert _find_esp32_port([NATIVE], HOST)[1] == NATIVE[1]
    assert _find_esp32_port([NATIVE], CONTAINER)[1] == '303a:1001 USB JTAG/serial debug unit'
    assert _find_esp32_port([COM], CONTAINER)[1] == '1a86:55d3 USB Single Serial'


def test_unknown_ttyacm_is_still_taken():
    """Verify that launch still takes a ttyACM it can't identify, as it did before."""
    unknown = ('ttyACM0', 'usb-Unknown_Serial-if00', '', '', '')
    port, _, socket = _find_esp32_port([unknown], (False, False))
    assert (port, socket) == ('ttyACM0', '')


def test_ttyusb_is_never_the_motor_board():
    """Verify that the lidar's CP2102 and a CH340 aren't taken for the motor board."""
    # Both sockets of the ESP32-S3 are ttyACM. A CH340 on ttyUSB, like the ESP32-CAM's, shares
    # the CH343's vendor ID, and opening its port resets that board.
    for view in (HOST, BY_ID_ONLY, CONTAINER):
        assert _find_esp32_port([LIDAR, CH340], view) == (None, None, ''), view
    assert _find_esp32_port([], HOST) == (None, None, '')


if __name__ == '__main__':
    test_esp32_native_usb_comes_before_its_com_socket()
    test_esp32_com_socket_alone_is_flagged()
    test_esp32_port_is_named_for_the_audit()
    test_unknown_ttyacm_is_still_taken()
    test_ttyusb_is_never_the_motor_board()
