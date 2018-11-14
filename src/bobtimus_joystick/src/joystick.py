#!/usr/bin/env python

"""
Joystick node to allow controlling the car with a gaming controller
"""

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool
import math


class Joystick():
    def __init__(self):
        self.subscribe()
        self.speed = 0.0
        self.steering_angle = 0.0
        self.buttons = None

        # init ros_node
        rospy.init_node('joystick_control', anonymous=True)

        # init publisher
        self.pub_speed = rospy.Publisher("/bobtimus/speed/command", Float64, queue_size=1)
        self.pub_steering = rospy.Publisher("bobtimus/steer/command", Float64, queue_size=1)
        self.pub_horn = rospy.Publisher("bobtimus/can/horn", Bool, queue_size=1)

    def callback_controller(self, data):
        """read controller input"""
        self.set_speed(data.axes[1])
        self.set_steering(data.axes[3])
        self.buttons = data.buttons

    def set_speed(self, new_speed):
        self.speed = new_speed

    def set_steering(self, new_steering):
        self.steeringAngle = new_steering

    def subscribe(self):
        # init subscriber
        rospy.Subscriber("joy", Joy, self.callback_controller)

    def publish(self):
        """Calculate values and publish them"""

        # react to button presses
        if self.buttons is not None:
            if self.button[0]:
                # (a) Button pressed -> honk the horn
                self.pub_horn.publish(True)
            else:
                # turn off horn again
                self.pub_horn.publish(False)

            if self.buttons[1]:
                # (b) Button pressed -> break
                self.speed = 0
                self.pub_acc.publish(Float64(0))

        # control speed (outputting 10m/s max equals 36km/h)
        speed_out = self.speed
        self.pub_speed.publish(Float64(speedOut))

        # control steering (max pi / 6.0 or 30 degrees)
        steer_out = self.steeringAngle * math.pi / 6.0
        self.pub_steering.publish(Float64(steer_out))

        rospy.loginfo("JoystickSpeed: " + str(round(speed_out, 3)) + "   JoystickSteeringAngle: " + str(
            round(steer_out, 3)))


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
