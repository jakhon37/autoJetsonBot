#!/usr/bin/env python3
"""Save SLAM's map through Nav2's map saver, and check what it wrote.

Runs in the robot's container, where ./robot.sh save-map, stop, nav and map2nav call it:

    python3 tools/save_map.py /autonomous_ROS/maps/<name>

Mapping mode keeps the saver running (main.launch.py), with its wait and thresholds from the
Nav2 params file. Exits 0 when <name>.yaml and <name>.pgm were written and load unknown cells as
unknown, and 1 otherwise, with the reason.
"""

import os
import sys
import time

import yaml

SERVICE = '/map_saver/save_map'
# How Nav2's map_server reads an unknown pixel (205). A free_thresh above it loads it as free.
UNKNOWN_OCCUPANCY = 1.0 - 205 / 255.0
FIND_TIMEOUT = 20.0  # s. A new node took about 3 s to find the others on the busy Jetson
ANSWER_TIMEOUT = 40.0  # s. The saver itself waits up to 15 s for /map


def pgm_counts(path):
    """Return a binary PGM's width, height and occupied, unknown and free pixel counts."""
    with open(path, 'rb') as f:
        data = f.read()
    fields, pos = [], 0
    while len(fields) < 4:  # P5, width, height, maxval; '#' starts a comment
        if data[pos:pos + 1].isspace():
            pos += 1
        elif data[pos:pos + 1] == b'#':
            pos = data.index(b'\n', pos)
        else:
            end = pos
            while not data[end:end + 1].isspace():
                end += 1
            fields.append(data[pos:end])
            pos = end
    if fields[0] != b'P5':
        raise ValueError(f'{path} is not a binary PGM')
    width, height = int(fields[1]), int(fields[2])
    pixels = data[pos + 1:pos + 1 + width * height]
    return width, height, pixels.count(0), pixels.count(205), pixels.count(254)


def call_saver(path):
    """Ask the map saver to save /map as path: return None if it did, 'no answer', or why not."""
    import rclpy
    from nav2_msgs.srv import SaveMap

    rclpy.init()
    node = rclpy.create_node(f'save_map_{os.getpid()}')
    try:
        client = node.create_client(SaveMap, SERVICE)
        if not client.wait_for_service(timeout_sec=FIND_TIMEOUT):
            return (f'{SERVICE} did not answer in {FIND_TIMEOUT:.0f} s. The map saver runs only '
                    'in mapping mode.')
        # free_thresh and occupied_thresh stay 0, so the saver uses its own.
        request = SaveMap.Request(map_topic='map', map_url=path, image_format='pgm',
                                  map_mode='trinary')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=ANSWER_TIMEOUT)
        if not future.done():
            return 'no answer'
        if not future.result().result:
            return ('The map saver failed. Its log, ./robot.sh logs, says why: no /map within '
                    'its wait, or a folder it cannot write.')
        return None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def written_since(path, started):
    return os.path.exists(path) and os.path.getmtime(path) >= started - 1.0


def main(argv):
    if len(argv) != 2 or argv[1].startswith('-'):
        print(__doc__, file=sys.stderr)
        return 2
    path = argv[1]
    started = time.time()
    error = call_saver(path)
    written = all(written_since(path + ext, started) for ext in ('.yaml', '.pgm'))
    if error == 'no answer':
        if written:
            # A server's answer is lost if it hasn't yet matched a new client's reader ("failed to
            # send response" in its log), as the saver's answer to its lifecycle manager once was
            # in Gazebo. The files show whether the save itself worked.
            print(f'The map saver gave no answer in {ANSWER_TIMEOUT:.0f} s, but wrote both files.')
            error = None
        else:
            error = f'The map saver gave no answer in {ANSWER_TIMEOUT:.0f} s.'
    if error is not None:
        print(f'Map not saved as {path}: {error}', file=sys.stderr)
        return 1
    if not written:
        print(f'The map saver said it saved {path}, but its .yaml and .pgm were not both written.',
              file=sys.stderr)
        return 1
    with open(path + '.yaml', 'r') as f:
        meta = yaml.safe_load(f)
    if not meta.get('free_thresh', 1.0) < UNKNOWN_OCCUPANCY:
        print(f"Saved {path}, but its free_thresh {meta.get('free_thresh')} loads unknown cells "
              "as free. Set 0.196 in it and in the map saver's free_thresh_default.",
              file=sys.stderr)
        return 1
    width, height, occupied, unknown, free = pgm_counts(path + '.pgm')
    print(f'Saved {path}.yaml and .pgm: {width} x {height} cells of {meta["resolution"]} m, '
          f'{occupied:,} occupied, {free:,} free and {unknown:,} unknown.')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
