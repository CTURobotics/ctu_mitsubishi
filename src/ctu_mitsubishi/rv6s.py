#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-11-28
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from serial import Serial, EIGHTBITS, PARITY_EVEN, STOPBITS_TWO
import numpy as np
from numpy.typing import ArrayLike


class Rv6s:

    def __init__(self, port="/dev/ttyUSB0", baudrate=38400, debug=False):
        self._debug = debug
        self._connection = Serial(
            port,
            baudrate,
            bytesize=EIGHTBITS,
            parity=PARITY_EVEN,
            stopbits=STOPBITS_TWO,
            timeout=1,
        )
        self._initialized = False
        self.q_home = np.deg2rad([0, 0, 90, 0, 90, 0])
        self.q_min = np.deg2rad([-170, -92, -107, -160, -120, -360])
        self.q_max = np.deg2rad([170, 135, 166, 160, 120, 360])

    def __del__(self):
        """Stop robot and close the connection to the robot's control unit."""
        self.stop_robot()
        self.close_connection()

    def close_connection(self):
        """Close the connection to the robot's control unit."""
        if hasattr(self, "_connection") and self._connection is not None:
            self._connection.close()
            self._connection = None

    def stop_robot(self):
        """Stop the robot and perform a safe shutdown."""
        if self._initialized:
            self._send_and_receive("1;1;HOTMoveit.MB4;M1=0\r")
            self._send_and_receive("1;1;STOP\r")
            self._send_and_receive("1;1;SLOTINIT\r")
            self._initialized = False

    def _send_and_receive(self, msg: str) -> str:
        """Send a message to the robot and return the response."""
        if self._debug:
            print("Sending message", msg)
        self._connection.write(msg.encode("utf-8"))
        res = self._connection.readline().decode("utf-8")
        if self._debug:
            print("Received message", res)
            print("----")
        return res

    def initialize(self):
        """Performs robot initialization."""
        if self._initialized:
            return
        res = self._send_and_receive("1;1;CNTLON\r")
        assert res.startswith("QoK"), f"Unexpected response while strating: {res}"

        res = self._send_and_receive("1;1;OVRD=25\r")
        assert res.startswith("QoK25"), f"Unexpected response while setting OVRD: {res}"

        res = self._send_and_receive("1;1;RUNMoveit;1\r")
        assert res.startswith("QoK"), f"Unexpected response while starting prog: {res}"

        self._initialized = True

    def get_q(self) -> np.ndarray:
        """Get current joint configuration [rad]."""
        assert self._initialized, "You need to initialize the robot before moving it."
        res = self._send_and_receive("1;1;JPOSF\r")
        assert res.startswith("QoK"), f"Unexpected response: {res}"
        res = res[3:]
        q = np.deg2rad(np.array([float(val) for val in res.split(";")[1:13:2]]))
        assert len(q) == 6, f"Unexpected number of joint values: {q}"
        return q

    def move_to_q(self, q: ArrayLike):
        """Move robot to the given joint configuration [rad] using coordinated movement.
        Initialization has be called before to set up the robot."""
        assert self._initialized, "You need to initialize the robot before moving it."
        assert self.in_limits(q), "Joint limits violated."
        msg = "1;1;HOTMoveit;J1=("
        for val in np.rad2deg(q):
            msg += f"{val:.2f},"
        msg += ")\r"
        res = self._send_and_receive(msg)
        assert res.lower().startswith("qok"), f"Unexpected response: {res}"

    def soft_home(self):
        """Move robot to the home position using move to q function."""
        self.move_to_q(self.q_home)

    def in_limits(self, q: ArrayLike) -> bool:
        """Return whether the given joint configuration is in joint limits."""
        return np.all(q >= self.q_min) and np.all(q <= self.q_max)

    def fk(self, q: ArrayLike) -> np.ndarray:
        """Compute forward kinematics for the given joint configuration [rad].
        Return pose of the end-effector in the base frame. Homogeneous transformation
        matrix (4x4) is returned."""
        pass

    # fk/ik
