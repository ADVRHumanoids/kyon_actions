"""MAT loading and joint ordering shared by live replay and simulation tools."""
from pathlib import Path

import numpy as np
from scipy.io import loadmat


JOINTS = [f'{joint}_{leg}' for leg in range(1, 5)
          for joint in ('hip_roll', 'hip_pitch', 'knee_pitch')]


def package_directory():
    source = Path(__file__).resolve().parent
    if (source / 'package.xml').is_file():
        return source
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory('kyon_actions'))


def load_trajectory(path):
    data = loadmat(path)
    dt = float(data['dt'].item())
    if not np.isfinite(dt) or dt <= 0:
        raise ValueError('dt must be a finite positive scalar')
    n = data['a'].shape[1]
    for key, shape in [('q', (19, n + 1)), ('v', (18, n + 1)),
                       ('a', (18, n)), ('tau', (18, n))]:
        if data[key].shape != shape or not np.isfinite(data[key]).all():
            raise ValueError(f'{key} must contain finite values with shape {shape}')
    if n < 1 or float(data['ns'].item()) != n:
        raise ValueError('ns must match the number of control samples')
    if not np.allclose(np.linalg.norm(data['q'][3:7], axis=0), 1, atol=1e-3):
        raise ValueError('Base quaternions must have unit length')
    return data, dt, n


def joint_feedback(message):
    indices = [message.name.index(joint) for joint in JOINTS]
    return [np.asarray(getattr(message, field), dtype=float)[indices]
            for field in ('link_position', 'link_velocity', 'effort')]
