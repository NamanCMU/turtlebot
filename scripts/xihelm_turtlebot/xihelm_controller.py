from threading import Thread, Lock

import rospy
from geometry_msgs.msg import Twist
from xihelm_pid import XihelmPID


class XihelmController(Thread):
    """This class runs the controller in a thread.

    It gets the updated orientation of the robot, appends it
    to the list and calls the PID loop function in a thread.
    """
    def __init__(self, cmdvel_pub):
        """The constructor for the Xihelm Controller class."""
        super(XihelmController, self).__init__()

        # List of orientation values and its max size
        self._orientation = []
        self._maxsize = 3

        # cmd vel publisher
        self._cmdvel_publisher = cmdvel_pub

        # Mode of the robot
        self._polarismode = True

        # Random velocity to publish to move the robot around
        self._linearx = 0.1
        self._angularz = 1.0

        self._mutex = Lock()

        # PID object
        self._pidobj = XihelmPID(self._cmdvel_publisher)
        self._flag = True

        self.start()

    def stop(self):
        """Thread's stop function."""
        rospy.loginfo("Thread ended.")
        self._flag = False
        self.join()

    def _change_mode(self):
        """Changes the mode of the robot operation."""
        self._polarismode = not self._polarismode
        print("Polaris Mode: ", self._polarismode)

    def _update_orientation(self, angle):
        """Get the updated orientation of the robot."""
        if len(self._orientation) == self._maxsize:
            self._orientation.pop()
        self._orientation.append(angle)

    def run(self):
        """Thread's run function."""
        while self._flag:
            if self._polarismode:
                if len(self._orientation) != 0:
                    self._mutex.acquire()

                    # Obtain yaw
                    angles = self._orientation.pop(0)
                    yaw = angles[2]

                    # Call PID loop
                    self._pidobj._pid_run(yaw)

                    self._mutex.release()
            else:
                # Create a random velocity message and publish
                random_cmdvel = Twist()
                random_cmdvel.linear.x = self._linearx
                random_cmdvel.angular.z = self._angularz

                self._cmdvel_publisher.publish(random_cmdvel)
