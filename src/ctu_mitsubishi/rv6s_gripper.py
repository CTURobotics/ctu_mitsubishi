#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-12-4
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

from papouch import Quido


class Rv6sGripper:

    def __init__(self, dev='/dev/ttyUSB_Quido'):
        self._quido = Quido()
        self._quido.connect_usb(dev)

    def open(self):
        self._quido.set_output(1, True)
        self._quido.set_output(1, False)

    def close(self):
        self._quido.set_output(2, True)
        self._quido.set_output(2, False)


#
# # initialize Quido ETH
# q = Quido()
# q.connect_usb('/dev/ttyUSB0')
#
# # get info about the device
# q.get_name()  # => [u'Quido USB 4/4', u'v0253.03.35', u'f66 97', u't']
#
# # set output
# q.set_output(1, True)  # => True
#
# # $ quido-cli --conn usb:/dev/ttyUSB_Quido seto 1H # to open the gripper
# # $ quido-cli --conn usb:/dev/ttyUSB_Quido seto 1L # set channel 1 low
# #
# # $ quido-cli --conn usb:/dev/ttyUSB_Quido seto 2H # to close the gripper
# # $ quido-cli --conn usb:/dev/ttyUSB_Quido seto 2L # set channel 2 low
