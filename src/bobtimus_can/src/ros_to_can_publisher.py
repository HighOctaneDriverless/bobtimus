#!/usr/bin/env python

"""
Subclasses of this abstract class store values from CAN messages and publish them to ROS topics
"""

import can
from abc import ABCMeta, abstractmethod


class RosToCanPublisher(object):
    __metaclass__ = ABCMeta

    def __init__(self, can_bus):
        self.can_bus = can_bus

    def send_message(self, arbitration_id, data, extended, timeout=0.2):
        """
        Sends a message on the CAN bus

        :param arbitration_id: id of the CAN message
        :param extended: defines whether message is of extended format or not
        :param data: message content
        :param timeout:  wait up to this many seconds for message to be ACK:ed or for transmit queue to be ready
                         depending on driver implementation. If timeout is exceeded, an exception will be raised
                         (default=0.2)
        """
        message = can.Message(arbitration_id=arbitration_id, data = data, extended_id=extended)

                
        self.can_bus.send(message, timeout)

    @abstractmethod
    def publish_values(self):
        """
        publishes all saved values to the CAN bus
        """
        pass

    @abstractmethod
    def subscribe(self):
        """
        subscribes to all important ros nodes
        """
        pass
