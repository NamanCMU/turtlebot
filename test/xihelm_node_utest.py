#!/usr/bin/env python

import unittest
from xihelm_turtlebot import xihelm_transformation
from sensor_msgs.msg import Imu

PKG = 'xihelm_turtlebot'


class TestTransformations(unittest.TestCase):
    """This class performs the unit testing for transformation function."""
    xihelm_object = xihelm_transformation.XihelmTransformation()

    def test_quaterntion_euler_transformation(self):
        """Test the transformation from quaternion to Euler angles."""
        # Fill up the IMU message
        imu_msg = Imu()
        imu_msg.orientation.x = 0.0
        imu_msg.orientation.y = 0.0
        imu_msg.orientation.z = 0.707
        imu_msg.orientation.w = 0.707

        # Call the transform function
        xihelm_euler_angle = self.xihelm_object._transform(imu_msg)

        # Ground truth Euler angles
        correct_euler_angle = [0.0, 0.0, 1.57]

        self.assertAlmostEqual(xihelm_euler_angle[0], correct_euler_angle[0], places=2)
        self.assertAlmostEqual(xihelm_euler_angle[1], correct_euler_angle[1], places=2)
        self.assertAlmostEqual(xihelm_euler_angle[2], correct_euler_angle[2], places=2)


if __name__ == '__main__':
    import rostest
    rostest.rosrun(PKG, 'test_transformations', TestTransformations)
