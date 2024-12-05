# !/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np
import unittest
from ctu_mitsubishi import Rv6s


class TestKinematics(unittest.TestCase):

    def test_fk0(self):
        q = np.deg2rad([0, 0, 90, 0, 0, 0])
        r = Rv6s(port=None)
        pose = r.fk(q)
        exp_z = 0.73
        exp_x = 0.485
        exp_y = 0
        theta = np.pi / 2
        exp_rot = np.array(
            [
                [np.cos(theta), 0, np.sin(theta)],
                [0, 1, 0],
                [-np.sin(theta), 0, np.cos(theta)],
            ]
        )
        np.testing.assert_allclose(pose[:3, 3], [exp_x, exp_y, exp_z], atol=1e-6)
        np.testing.assert_allclose(pose[:3, :3], exp_rot, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
