import rospy
import tf
from sensor_msgs.msg import Imu


class XihelmTransformation(object):
    """This class performs the task of transforming the input quaternion
    to euler angles.
    """
    def __init__(self):
        """The constructor of the Xihelm Transformation class."""
        self._imumsg = Imu()

    def _transform(self, imu_msg):
        """This functions performs the transformation."""
        # TODO Pool/ Filter the IMU data temporally to remove the noise/ outliers
        self._imumsg = imu_msg
        quat = (self._imumsg.orientation.x, self._imumsg.orientation.y, self._imumsg.orientation.z,
                self._imumsg.orientation.w)
        euler_angle = tf.transformations.euler_from_quaternion(quat)
        return euler_angle
