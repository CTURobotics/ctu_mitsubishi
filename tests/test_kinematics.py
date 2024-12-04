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

    def test_fk_zeros(self):
        r = Rv6s(port=None)
        pose = r.fk([0, 0, 0, 0, 0, 0])
        exp_translation = [
            0.0,
            0.0,
            0.0,
        ]
        np.testing.assert_allclose(pose[:3, 3], exp_translation, atol=1e-6)
        np.testing.assert_allclose(pose[:3, :3], np.eye(3), atol=1e-6)

if __name__ == "__main__":
    unittest.main()
