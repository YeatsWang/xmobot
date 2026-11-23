"""Run with Python 3 in an environment with xacro installed."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import xacro


MODEL = Path(__file__).resolve().parents[1] / 'urdf' / 'xmobot.urdf.xacro'
WHEELS = ('front_left', 'front_right', 'rear_left', 'rear_right')
DRIVES = ('4wis', 'diff_drive', 'mecanum', 'ackermann_front', 'ackermann_rear')


def expand(**args):
    document = xacro.process_file(str(MODEL), mappings={k: str(v) for k, v in args.items()})
    return ET.fromstring(document.toxml())


def origin(joint):
    return tuple(map(float, joint.find('origin').get('xyz').split()))


class MountGeometry(unittest.TestCase):
    def test_ros2_control_matches_installed_joints(self):
        for drive in DRIVES:
            for count in range(3):
                with self.subTest(drive=drive, count=count):
                    root = expand(drive_type=drive, arm_count=count, control_config='/tmp/test.yaml')
                    expected = {j.get('name') for j in root.findall('joint') if j.get('type') != 'fixed'}
                    hardware = root.findall('ros2_control/joint')
                    self.assertCountEqual([j.get('name') for j in hardware], expected)
                    for joint in hardware:
                        mode = 'velocity' if joint.get('name').endswith('_wheel_joint') else 'position'
                        self.assertEqual(joint.find('command_interface').get('name'), mode)
                    plugin = root.find("gazebo/plugin[@filename='libgazebo_ros2_control.so']")
                    self.assertEqual(plugin.findtext('parameters'), '/tmp/test.yaml')

    def test_ground_contact_and_front_camera(self):
        for drive in DRIVES:
            for length, height, radius in ((.32, .21, .05), (.84, .05, .15), (.50, .10, .08)):
                with self.subTest(drive=drive, length=length, height=height, radius=radius):
                    root = expand(drive_type=drive, robot_length=length, robot_height=height, wheel_radius=radius)
                    joints = {j.get('name'): j for j in root.findall('joint')}
                    body_z = origin(joints['base_joint'])[2]
                    self.assertAlmostEqual(body_z - height / 2, 2 * radius + .03)
                    for wheel in WHEELS:
                        steer = joints.get(wheel + '_steering_joint')
                        axle = body_z + origin(joints[wheel + '_wheel_joint'])[2]
                        if steer is not None:
                            axle += origin(steer)[2]
                        collision = root.find(f"link[@name='{wheel}_wheel_link']/collision/geometry/cylinder")
                        self.assertAlmostEqual(axle - float(collision.get('radius')), 0)
                    cx, cy, cz = origin(joints['camera_joint'])
                    self.assertAlmostEqual(cx - .025, length / 2)
                    self.assertEqual((cy, cz), (0, 0))

    def test_mount_heights_follow_the_deck_in_ground_coordinates(self):
        for thickness in (.05, .10, .21):
            for count in (0, 1, 2):
                with self.subTest(thickness=thickness, count=count):
                    root = expand(robot_height=thickness, wheel_radius=.08, arm_count=count)
                    body_z = origin(root.find("joint[@name='base_joint']"))[2]
                    deck_z = 2 * .08 + .03 + thickness
                    lidar_z = origin(root.find("joint[@name='lidar_joint']"))[2]
                    self.assertAlmostEqual(body_z + lidar_z - .025, deck_z)
                    for side in ('left', 'right'):
                        joint = root.find(f"joint[@name='{side}_joint_1']")
                        if joint is not None:
                            self.assertAlmostEqual(body_z + origin(joint)[2], deck_z)

    def test_adjustable_mounts(self):
        def position(count, **args):
            root = expand(arm_count=count, **args)
            return origin(root.find("joint[@name='left_joint_1']"))[:2]

        self.assertEqual(position(1), (-.125, 0))
        self.assertEqual(position(1, arm_mount_x=-.16), (-.16, 0))
        self.assertEqual(position(1, arm_mount_x=0), (-.102, 0))
        self.assertEqual(position(1, arm_mount_x=-1), (-.2, 0))
        self.assertEqual(position(2, arm_mount_x=.075, arm_spacing=.22), (.075, .11))
        x, y = position(2, robot_width=.25, arm_mount_x=0, arm_spacing=.13)
        self.assertAlmostEqual(x*x + y*y, .102**2)

    def test_arm_layouts_and_transmissions(self):
        for count in range(3):
            with self.subTest(count=count):
                root = expand(arm_count=count, robot_length=.32, robot_width=.25, robot_height=.21)
                joints = {j.get('name'): j for j in root.findall('joint')}
                arm_joints = [joints[side + '_joint_1'] for side in ('left', 'right') if side + '_joint_1' in joints]
                self.assertEqual(len(arm_joints), count)
                lidar_x, lidar_y, lidar_z = origin(joints['lidar_joint'])
                self.assertEqual((lidar_x, lidar_y), (0, 0))
                self.assertAlmostEqual(lidar_z - .025, .21 / 2)
                for joint in arm_joints:
                    x, y, z = origin(joint)
                    self.assertAlmostEqual(z, .21 / 2)
                    self.assertGreater((x - lidar_x)**2 + y**2, (.05 + .05)**2)
                if count == 1:
                    self.assertAlmostEqual(origin(arm_joints[0])[0], -.102)
                    self.assertEqual(origin(arm_joints[0])[1], 0)
                if count == 2:
                    self.assertAlmostEqual(origin(arm_joints[0])[1], -origin(arm_joints[1])[1])
                for tx in root.findall('transmission/joint'):
                    self.assertIn(tx.get('name'), joints)
                for side, installed in [('left', count >= 1), ('right', count == 2)]:
                    self.assertEqual(root.find(f"transmission/joint[@name='{side}_joint_1']") is not None, installed)
        self.assertIsNone(expand(use_arms='false', arm_count=2).find("link[@name='left_link_1']"))
        self.assertIsNotNone(expand().find("link[@name='right_link_1']"))


if __name__ == '__main__':
    unittest.main()
