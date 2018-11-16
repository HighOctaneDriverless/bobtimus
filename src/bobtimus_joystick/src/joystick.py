#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
import math

"""
Node to allow controlling the racecar with a common gaming controller.
It takes the input from the joy node and performs actions based on the pressed buttons
"""


class Joystick():
    def __init__(self):
        self.subscribe()
        self.speed = 0.0
        self.steeringAngle = 0.0
        self.buttons = None

        # init ros_node
        rospy.init_node('joystick_control', anonymous=True)

        # init publisher
        self.pub_speed = rospy.Publisher("/bobtimus/speed/", Float64, queue_size=1)
        self.pub_steering = rospy.Publisher("bobtimus/steer/", Float64, queue_size=1)
        self.pub_horn = rospy.Publisher("bobtimus/can/horn", Bool, queue_size=1)

    def callback_controller(self, data):
        """Gets called when new controller data is available"""

        # RT Button
        self.speed = data.axes[5]
        # Left Joystick X Axis
        self.steeringAngle = data.axes[0]
        # Rest of the Buttons
        self.buttons = data.buttons

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("joy", Joy, self.callback_controller)

    def publish(self):
        if self.buttons is not None:
            """button was pressed"""

            if self.buttons[0]:
                # (a) Button pressed
                self.pub_horn.publish(True)
            else:
                self.pub_horn.publish(False)

            if self.buttons[1]:
                # (b) Button pressed
                self.speed = -1;
                self.pub_speed.publish(Float64(0))

        # Calculate speed, convert [1,-1] range to [0,1] range
        speed_out = 1 - ((self.speed + 1) / 2)
        self.pub_speed.publish(Float64(speed_out))

        # Calculate steering, with a maximum of 25 degrees in each direction
        steer_out = self.steeringAngle * math.radians(25)
        self.pub_steering.publish(Float64(steer_out))


def main():
    joystick = Joystick()
    rospy.loginfo("Joystick started")
    rate = rospy.Rate(100)
    while not rospy.is_shutdown():
        joystick.publish()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
