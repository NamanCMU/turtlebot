#!/usr/bin/env python

import rospy
from std_msgs.msg import String

import sys
import termios
import tty
import select


def read_keyboard(key_timeout, settings):
    """Read the key pressed from the keyboard.
    Reference: http://wiki.ros.org/turtlebot_teleop
    """
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)

    try:
        # Initialize the node
        rospy.init_node('xihelm_keyboard_control_node', anonymous=True)

        # Publishes the key pressed by the user
        keyboard_publisher = rospy.Publisher('/xihelm/keyboard', String, queue_size=2)

        print("Press 't' to change the mode between")
        print("1.Random motion of the robot.")
        print("2.Orienting the robot to the Polaris.")
        print("Default is 2.Orienting the robot to the Polaris.")

        rate = rospy.Rate(50)  # 50 Hz

        while not rospy.is_shutdown():
            # Read the key
            ip_key = read_keyboard(1.0, settings)

            # Mode changes when 't' is pressed
            if(ip_key == 't'):
                print("Mode changed: ", ip_key)

            # Condition to exit: Ctrl C
            if(ip_key == '\x03'):
                break

            # Publish the string message
            ip_key = str(ip_key)
            keyboard_publisher.publish(ip_key)

            rate.sleep()

    except Exception as e:
        print("Exception occurred: ", e)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
