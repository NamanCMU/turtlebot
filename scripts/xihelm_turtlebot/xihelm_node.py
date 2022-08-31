#!/usr/bin/env python

import time

import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from xihelm_turtlebot.msg import NodeStatus

from xihelm_controller import XihelmController
from xihelm_transformation import XihelmTransformation


class XihelmNode(object):
    """This class performs the task of orienting the robot to the North star.

    This class subscribes to the IMU from turtlebot and uses its Yaw to correct
    the orientation.
    """
    def __init__(self):
        """The constructor of the Xihelm Node class."""
        # TODO move these topic names to the launch file
        # Define the subscriber to the imu data
        self._imu_subscriber = rospy.Subscriber('/imu', Imu, self._imu_callback)

        # Define the subscriber to the keyboard data
        self._keyboard_subscriber = rospy.Subscriber('/xihelm/keyboard', String, self._keyboard_callback)
        # The key pressed
        self._ipkey = ""

        # Define the publisher of the cmd vel
        self._cmdvel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)

        # Health Status publisher
        self._xihelm_healthstatus_publisher = rospy.Publisher('/node/status',
                                                              NodeStatus, queue_size=2)
        # Health status message
        self._xihelm_statusmsg = NodeStatus()

        # Controller object
        self._xihelmcontrollerobj = XihelmController(self._cmdvel_publisher)

        # Transformation object
        self._xihelmtransformationobj = XihelmTransformation()

        # Wait for some time till all publishers and subscribers are ready
        time.sleep(0.100)

        # Update the status code - RUNNING
        status_code = "RUNNING"
        self._health_status_update(0, status_code)
        rospy.loginfo(status_code)

    def _keyboard_callback(self, key_msg):
        """Keyboard callback function."""
        self._ipkey = key_msg.data

        # If 't' is pressed, change the mode
        if(self._ipkey == "t"):
            self._xihelmcontrollerobj._change_mode()

    def _imu_callback(self, imu_msg):
        """IMU callback function."""
        # Transform and get the euler angle representation
        euler_angle = self._xihelmtransformationobj._transform(imu_msg)
        self._xihelmcontrollerobj._update_orientation(euler_angle)

    def _health_status_update(self, seq_no, status_code):
        """This updates the health status of the node."""
        self._xihelm_statusmsg.header.stamp = rospy.Time.now()
        self._xihelm_statusmsg.seq_no = seq_no
        self._xihelm_statusmsg.node_name = rospy.get_name()
        self._xihelm_statusmsg.node_status = status_code

        # Publishing the health status message/ update
        self._xihelm_healthstatus_publisher.publish(self._xihelm_statusmsg)


def main():
    """This is the main master function."""
    logging_level = rospy.DEBUG  # Define the logging level

    # Initializing the node
    rospy.init_node('xihelm_node', anonymous=True, log_level=logging_level)
    xihelmobj = XihelmNode()

    # Continuous Spinning
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting Down")
        status_code = "SHUTTING DOWN"
        xihelmobj._health_status_update(-1, status_code)
        xihelmobj._xihelmcontrollerobj.stop()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
