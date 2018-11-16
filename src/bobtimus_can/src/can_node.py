#!/usr/bin/env python

"""
Interface between can bus and ros nodes. It intercepts CAN messages and publishes them to ROS and sends messages
from ROS topics on the CAN bus
"""

import can
import rospy

from horn_publisher import HornPublisher


class CanNode(can.Listener):
    # filters that should be applied to CAN messages
    FILTERS = [{"can_id": 0x6A0, "can_mask": 0x7F8, "extended": False}]  # MPU (6A0-6A7)

    horn_publisher = None

    bus = None

    def __init__(self):
        self.can_setup()
        self.ros_setup()

    def can_setup(self):
        """
        Initialize the CAN bus
        """
        # create a bus instance
        self.bus = can.Bus(interface='socketcan', channel='can0', receive_own_messages=False)

        # setup filter for can messages
        can_filters = self.FILTERS
        self.bus.set_filters(can_filters)

        # or use an asynchronous notifier
        can.Notifier(self.bus, [self])

    def ros_setup(self):
        """
        Initialize ROS
        """
        # init ros_node
        rospy.init_node('can_node', anonymous=True)

        # init ros publisher
        self.horn_publisher = HornPublisher(self.bus)

    def on_message_received(self, msg):
        """
        Method gets called for every received can message that passed the filter
        :param msg: can.Message object that bundles CAN message details and data
        """
        pass

    def publish_to_ros(self):
        """
        publishes received values from CAN to ROS
        """
        pass

    def publish_to_can(self):
        """
        publishes received values from ROS to CAN
        """
        self.horn_publisher.publish_values()
        pass


def main():
    can_node = CanNode()
    rate = rospy.Rate(100)

    while not rospy.is_shutdown():
        can_node.publish_to_ros()
        can_node.publish_to_can()
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
