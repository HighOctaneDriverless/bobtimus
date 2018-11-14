#!/usr/bin/env python

"""
ROS to CAN publisher for the car horn
"""

from ros_to_can_publisher import RosToCanPublisher
from std_msgs.msg import Bool, Int8, Int16
import rospy


class HornPublisher(RosToCanPublisher):

    def __init__(self, can_bus):
        super(HornPublisher, self).__init__(can_bus)
        self.horn_status = False
        self.subscribe()

    def subscribe(self):
        rospy.Subscriber("bobtimus/can/horn", Bool, self.set_horn_status)

    def publish_values(self):
        if self.horn_status:
            print "0x242" + str(int(self.horn_status))
            self.send_message(0x242, [int(self.horn_status), 0, 0, 0, 0, 0, 0, 0], False)

    def set_horn_status(self, data):
        self.horn_status = data.data
