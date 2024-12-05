#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-11-28
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#
from serial import Serial, EIGHTBITS, PARITY_EVEN, STOPBITS_TWO
import numpy as np
from numpy.typing import ArrayLike

from ctu_mitsubishi.utils import circle_circle_intersection


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

        self.dh_theta_off = np.deg2rad([0, -90, -90, 0, 0, 180])
        self.dh_a = np.array([85, 280, 100, 0, 0, 0]) / 1000.0
        self.dh_d = np.array([350, 0, 0, 315, 0, 85]) / 1000.0
        self.dh_alpha = np.deg2rad([-90, 0, -90, 90, -90, 0])

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

    @staticmethod
    def _rx(angle: float) -> np.ndarray:
        """Return SE3 transformation that rotates around x-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

    @staticmethod
    def _ry(angle: float) -> np.ndarray:
        """Return SE3 transformation that rotates around x-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

    @staticmethod
    def _rz(angle: float) -> np.ndarray:
        """Return SE3 transformation that rotates around z-axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    @staticmethod
    def _t(tx: float = 0.0, ty: float = 0.0, tz: float = 0.0) -> np.ndarray:
        """Return SE3 transformation that translates along x-axis."""
        return np.array([[1, 0, 0, tx], [0, 1, 0, ty], [0, 0, 1, tz], [0, 0, 0, 1]])

    @staticmethod
    def dh_to_se3(d: float, theta: float, a: float, alpha: float) -> np.ndarray:
        """Compute SE3 matrix from DH parameters."""
        return (
            Rv6s._t(tz=d)
            @ Rv6s._rz(angle=theta)
            @ Rv6s._t(tx=a)
            @ Rv6s._rx(angle=alpha)
        )

    def fk(self, q: ArrayLike) -> np.ndarray:
        """Compute forward kinematics for the given joint configuration [rad].
        Return pose of the end-effector in the base frame. Homogeneous transformation
        matrix (4x4) is returned. The pose represents the flange pose w.r.t. base
        of the robot."""
        pose = np.eye(4)
        for d, a, alpha, theta, qi in zip(
            self.dh_d, self.dh_a, self.dh_alpha, self.dh_theta_off, q
        ):
            pose = pose @ self.dh_to_se3(d, qi + theta, a, alpha)
        return pose

    def _ik_5th_joint_pos(self, position: ArrayLike, q) -> list[np.ndarray]:
        """Compute inverse kinematics for the given position of the 5th joint
        (flange w.r.t. base) in the base frame.
        Return list of joint configurations [rad] for the first three joints.
        """
        position = np.asarray(position)
        assert position.shape == (3,), "Position must be 3D vector."
        x, y, z = position

        res = []
        j1s = [np.atan2(y, x), np.atan2(-y, -x)]
        for j1 in j1s:
            pose_base_j2 = self.dh_to_se3(
                self.dh_d[0], j1, self.dh_a[0], self.dh_alpha[0]
            ) @ self._rz(-np.pi / 2)
            xy_j2 = (np.linalg.inv(pose_base_j2) @ np.array([x, y, z, 1]))[:2]
            r1 = self.dh_a[1]
            r2 = np.linalg.norm([self.dh_d[3], self.dh_a[2]])
            sols = circle_circle_intersection([0, 0], r1, xy_j2, r2)
            j2s = [np.arctan2(s[1], s[0]) for s in sols]

            for j2 in j2s:
                ang = np.atan2(self.dh_a[2], self.dh_d[3])
                pose_j2_j3 = self._rz(j2) @ self._t(tx=r1) @ self._rz(-ang)
                xy_j3 = (
                    np.linalg.inv(pose_j2_j3) @ np.array([xy_j2[0], xy_j2[1], 0, 1])
                )[:2]
                j3 = np.atan2(xy_j3[1], xy_j3[0])
                res.append(np.array([j1, j2, j3]))
        return res

    def ik(self, pose: ArrayLike) -> list[np.ndarray]:
        """Compute inverse kinematics for the given pose of the end-effector (flange
        w.r.t. base) in the base frame. Homogeneous transformation matrix (4x4) is
        expected.
        Return list of joint configurations [rad].
        """

        return []
