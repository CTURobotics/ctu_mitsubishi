#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-12-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from papouch import Quido


class Rv6sGripper:

    def __init__(self, dev="/dev/ttyUSB_Quido"):
        self._quido = Quido()
        self._quido.connect_usb(dev)

        self._open_channel = 2
        self._close_channel = 1

        self._quido.set_output(self._open_channel, False)
        self._quido.set_output(self._close_channel, False)

    def open(self, duration: float = 1.0):
        """Start opening the gripper and stop after duration in seconds."""
        self._quido.set_output(self._open_channel, True, duration=duration)

    def close(self, duration: float = 1.0):
        """Start closing the gripper and stop after duration in seconds."""
        self._quido.set_output(self._close_channel, True, duration=duration)

    def disconnect(self):
        """Stop controlling the gripper."""
        self._quido.set_output(self._open_channel, False)
        self._quido.set_output(self._close_channel, False)

    def __del__(self):
        self.disconnect()
