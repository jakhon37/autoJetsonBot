"""Find the motor board's and the lidar's serial ports, for launch, the bridge and robot.sh.

It imports no ROS, so all three can share it. Both of the ESP32-S3's USB sockets show up as
/dev/ttyACM*, but the firmware talks only on the native one, next to the RGB LED (Espressif's USB
JTAG/serial unit, 303a:1001). The one marked COM is a CH343 (1a86:55d3), where it sends nothing.
From 2026-11-11 to 11-23 the cable sat in the COM socket and the bridge connected to it without a
word, so the 2026-11-17 run got no wheel or IMU data. The robot's container has no
/dev/serial/by-id, so the USB IDs come from sysfs as well.
"""
import os

LIDAR_USB_ID = '10c4:ea60'  # the RPLidar A1's USB adapter, a Silicon Labs CP2102


def serial_by_id(dev='/dev'):
    """Return {name: device path} from /dev/serial/by-id, or {} where there's none."""
    root = os.path.join(dev, 'serial', 'by-id')
    mapping = {}
    if os.path.isdir(root):
        for name in sorted(os.listdir(root)):
            mapping[name] = os.path.realpath(os.path.join(root, name))
    return mapping


def read_usb_id(tty, sys_class_tty='/sys/class/tty'):
    """Return a tty's USB vendor:product and product name from sysfs, or '' if unknown."""
    path = os.path.realpath(os.path.join(sys_class_tty, tty, 'device'))
    for _ in range(3):  # the USB device is the interface's parent, one more level for ttyUSB
        if os.path.isfile(os.path.join(path, 'idVendor')):
            values = []
            for attr in ('idVendor', 'idProduct', 'product'):
                try:
                    with open(os.path.join(path, attr), 'r', errors='replace') as f:
                        values.append(f.read().strip())
                except OSError:
                    values.append('')
            return '{}:{} {}'.format(*values).strip()
        path = os.path.dirname(path)
    return ''


def esp32_socket(by_id_name, usb_id):
    """Say which of the motor board's USB sockets a serial device is: 'native', 'com' or ''."""
    vendor = usb_id.split(':')[0]
    name = by_id_name.lower()
    if vendor == '303a' or 'espressif' in name or 'jtag_serial' in name:
        return 'native'
    if vendor == '1a86' or any(s in name for s in ('1a86', 'qinheng', 'wch', 'usb_single_serial')):
        return 'com'
    return ''


def find_esp32_port(dev='/dev', sys_class_tty='/sys/class/tty'):
    """Return the motor board's port, a name for it, and its socket: 'native', 'com' or ''.

    Takes the native socket first, then the COM socket, which callers warn about, then any other
    ttyACM, and never a ttyUSB, where the lidar's CP2102 and CH340 boards sit.
    """
    by_id = {}
    for name, path in serial_by_id(dev).items():
        by_id.setdefault(path, name)
    try:
        ports = {os.path.realpath(os.path.join(dev, n)) for n in os.listdir(dev)
                 if n.startswith('ttyACM')}
    except OSError:
        ports = set()
    ports.update(p for p in by_id if os.path.basename(p).startswith('ttyACM'))
    rank = {'native': 0, 'com': 1, '': 2}
    found = []
    for path in sorted(ports):
        if not os.path.exists(path):
            continue
        usb_id = read_usb_id(os.path.basename(path), sys_class_tty)
        socket = esp32_socket(by_id.get(path, ''), usb_id)
        found.append((rank[socket], path, by_id.get(path) or usb_id or path, socket))
    if not found:
        return None, None, ''
    _, path, name, socket = min(found)
    return path, name, socket


def find_lidar_port(dev='/dev', sys_class_tty='/sys/class/tty'):
    """Return the lidar's port and a name for it, or (None, None).

    Only the RPLidar's CP2102 adapter, by its /dev/serial/by-id name or by its USB ID in sysfs,
    never just the first ttyUSB: robot.sh holds this port's DTR line to stop the lidar's motor,
    and held on a CH340 like the ESP32-CAM's programmer, DTR keeps that board in reset.
    """
    for name, path in serial_by_id(dev).items():
        low = name.lower()
        if ('cp210' in low or 'silicon_labs' in low) and os.path.exists(path):
            return path, name
    try:
        ttys = sorted(n for n in os.listdir(dev) if n.startswith('ttyUSB'))
    except OSError:
        ttys = []
    for tty in ttys:
        usb_id = read_usb_id(tty, sys_class_tty)
        if usb_id.startswith(LIDAR_USB_ID):
            return os.path.join(dev, tty), usb_id
    return None, None


def com_socket_warning(port, name):
    """Say that the motor board's cable is in its COM socket, and what to do about it."""
    return (f"the ESP32's cable is in its COM socket ({port}, {name}). The firmware sends "
            "nothing there, so no wheel or IMU data will come. Move the cable to the native USB "
            "socket next to the RGB LED, then restart the container (./robot.sh stop f) and "
            "start again.")
