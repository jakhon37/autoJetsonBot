"""Read a camera_info YAML as camera_calibration_parsers writes it. Imports no ROS."""

import yaml

# distortion coefficients each model takes
COEFFICIENTS = {'plumb_bob': 5, 'rational_polynomial': 8, 'equidistant': 4}


def load(path):
    """
    Return the calibration in a file, as a dict named like CameraInfo's fields.

    Keys: width, height, camera_name, distortion_model, d, k (9), r (9), p (12).
    Raises OSError if the file can't be read and ValueError if it isn't a calibration.
    """
    with open(path) as f:
        data = yaml.safe_load(f)
    try:
        def matrix(key, rows, cols):
            block = data[key]
            values = [float(v) for v in block['data']]
            if (block['rows'], block['cols']) != (rows, cols) or len(values) != rows * cols:
                raise ValueError(f'{key} should be {rows}x{cols}')
            return values

        model = str(data['distortion_model'])
        d = [float(v) for v in data['distortion_coefficients']['data']]
        if model in COEFFICIENTS and len(d) != COEFFICIENTS[model]:
            raise ValueError(f'{model} takes {COEFFICIENTS[model]} coefficients, not {len(d)}')
        return {
            'width': int(data['image_width']),
            'height': int(data['image_height']),
            'camera_name': str(data.get('camera_name', '')),
            'distortion_model': model,
            'd': d,
            'k': matrix('camera_matrix', 3, 3),
            'r': matrix('rectification_matrix', 3, 3),
            'p': matrix('projection_matrix', 3, 4),
        }
    except (KeyError, TypeError) as exc:
        raise ValueError(f'not a camera_info file: {exc!r}') from None
