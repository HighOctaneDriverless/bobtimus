#!/usr/bin/env python

"""
Subclasses of this abstract class store values from CAN messages and publish them to ROS topics
"""


class CanToRosPublisher:

    def publish_values(self):
        """
        publishes all saved values to ROS topics
        """
        pass

    def save_msg_values(self, msg):
        """
        stores received message values in variables
        :param msg: can.message object that stores the values that should be saved
        """
        pass
