import rospy
from geometry_msgs.msg import Twist


class XihelmPID(object):
    """This class performs the PID."""
    def __init__(self, cmdvel_pub):
        """The constructor for the PID class."""
        # cmd vel publisher
        self._cmdvel_pub = cmdvel_pub

        # cmd vel output
        self._output = Twist()

        # PID Values
        self._p = 2.0
        self._i = 0.0
        self._d = 0.0

    def _pid_run(self, yaw):
        """The step function to perform PID loop."""
        error = -yaw  # Error that PID needs to correct

        # Compute the correction and output cmd vel
        self._output.angular.z = min(0.5, self._p * error)

        # Publishing the velocity
        self._cmdvel_pub.publish(self._output)
