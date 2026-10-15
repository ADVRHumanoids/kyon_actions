import tempfile
import unittest
import argparse
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

import numpy as np
from scipy.io import loadmat, savemat

from replay_sim_ros2 import JOINTS, load_trajectory, configure_payload, payload_mass, joint_feedback


class TrajectoryValidationTest(unittest.TestCase):
    def test_feedback_uses_joint_names_instead_of_message_order(self):
        message = SimpleNamespace(name=list(reversed(JOINTS)),
                                  link_position=list(range(12)),
                                  link_velocity=list(range(20, 32)),
                                  effort=list(range(40, 52)))
        q, v, torque = joint_feedback(message)
        np.testing.assert_array_equal(q, np.arange(12)[::-1])
        np.testing.assert_array_equal(v, np.arange(20, 32)[::-1])
        np.testing.assert_array_equal(torque, np.arange(40, 52)[::-1])
        message.name.pop()
        with self.assertRaises(ValueError):
            joint_feedback(message)

    def test_payload_mass_and_box_inertia(self):
        urdf = ET.fromstring('''<robot>
          <link name="payload_pelvis">
            <inertial><mass value="20"/>
              <inertia ixx=".13" iyy=".13" izz=".13" ixy="0" ixz="0" iyz="0"/>
            </inertial>
            <visual><geometry><box size=".2 .2 .2"/></geometry></visual>
          </link>
          <joint name="pelvis_to_payload"><origin xyz="-.2 0 .17"/></joint>
        </robot>''')
        self.assertEqual(configure_payload(urdf, 30.), [-.2, 0., .17])
        self.assertEqual(float(urdf.find('link/inertial/mass').get('value')), 30.)
        for axis in ('ixx', 'iyy', 'izz'):
            self.assertAlmostEqual(float(urdf.find('link/inertial/inertia').get(axis)), .2)
        self.assertIsNotNone(urdf.find('link/collision/geometry/box'))

    def test_invalid_payload_mass(self):
        for value in ('-1', 'nan', 'inf'):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                payload_mass(value)

    def test_bundled_step_up_files(self):
        for path in (Path(__file__).parent / 'mat_files').glob('step_up*.mat'):
            with self.subTest(path=path):
                data, dt, n = load_trajectory(path)
                self.assertEqual(n, 507)
                self.assertAlmostEqual(n * dt, 20.28)
                self.assertEqual(data['q'].shape[1], n + 1)

    def test_reject_invalid_motion_before_connecting_to_controller(self):
        original = loadmat(Path(__file__).parent / 'mat_files/step_up_clearance.mat')
        invalid = [
            {'dt': np.array([[0.]])},
            {'v': original['v'][:, :-1]},
            {'tau': np.full_like(original['tau'], np.nan)},
            {'ns': np.array([[10]])},
            {'ns': np.array([[507.5]])},
            {'q': original['q'] * 2},
        ]
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.mat'
            for change in invalid:
                with self.subTest(field=next(iter(change))):
                    savemat(path, original | change)
                    with self.assertRaises(ValueError):
                        load_trajectory(path)


if __name__ == '__main__':
    unittest.main()
