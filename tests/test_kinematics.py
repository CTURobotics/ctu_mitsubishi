# !/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2024-10-31
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import numpy as np
import unittest
from ctu_mitsubishi import Rv6s
from ctu_mitsubishi.utils import circle_circle_intersection


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

    def test_circ_circ_intersection(self):
        ints = circle_circle_intersection([0, 0], 1, [1, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        if by > ay:
            ay, by = by, ay
        self.assertAlmostEqual(ax, 0.5)
        self.assertAlmostEqual(ay, np.sqrt(1 - 0.5**2))
        self.assertAlmostEqual(bx, 0.5)
        self.assertAlmostEqual(by, -np.sqrt(1 - 0.5**2))

        ints = circle_circle_intersection([0, 0], 1, [2, 0], 1)
        self.assertEqual(len(ints), 2)
        (ax, ay), (bx, by) = ints
        self.assertAlmostEqual(ax, 1.0)
        self.assertAlmostEqual(ay, 0.0)
        self.assertAlmostEqual(bx, 1.0)
        self.assertAlmostEqual(by, 0.0)

        ints = circle_circle_intersection([0, 0], 1, [0, 0], 1)
        self.assertEqual(len(ints), 2)
        a, b = ints
        self.assertAlmostEqual(np.linalg.norm(a), 1.0)
        self.assertAlmostEqual(np.linalg.norm(b), 1.0)

        ints = circle_circle_intersection([0, 0], 1, [5, 0], 1)
        self.assertEqual(len(ints), 0)

    def _assert_on_both_circles(self, ints, c0, r0, c1, r1, atol=1e-9):
        for p in ints:
            self.assertAlmostEqual(np.linalg.norm(p - c0), r0, delta=atol)
            self.assertAlmostEqual(np.linalg.norm(p - c1), r1, delta=atol)

    def test_circ_circ_intersection_nested(self):
        """One circle strictly inside the other: no intersection, no assert."""
        self.assertEqual(len(circle_circle_intersection([0, 0], 3, [0.5, 0], 1)), 0)
        self.assertEqual(len(circle_circle_intersection([0.5, 0], 1, [0, 0], 3)), 0)
        # concentric with different radii
        self.assertEqual(len(circle_circle_intersection([0, 0], 1, [0, 0], 2)), 0)

    def test_circ_circ_intersection_tangent_rounding(self):
        """Exactly tangent circles whose rounding used to trigger the assert."""
        c0, r0 = np.array([-0.766735510274243, 3.2770259382044173]), 2.514274904578052
        c1, r1 = np.array([5.002554891197656, 4.286136541335328]), 3.3426027572710515
        self.assertAlmostEqual(np.linalg.norm(c1 - c0), r0 + r1)
        ints = circle_circle_intersection(c0, r0, c1, r1)
        self.assertEqual(len(ints), 2)
        self._assert_on_both_circles(ints, c0, r0, c1, r1)
        np.testing.assert_allclose(ints[0], ints[1], atol=1e-6)

    def test_circ_circ_intersection_internal_tangent(self):
        """Smaller circle touching the bigger one from inside."""
        ints = circle_circle_intersection([0, 0], 3, [1, 0], 2)
        self.assertEqual(len(ints), 2)
        for p in ints:
            np.testing.assert_allclose(p, [3, 0], atol=1e-9)

    def test_circ_circ_intersection_tangent_random(self):
        """Random external and internal tangency: a single (duplicated) point."""
        rng = np.random.default_rng(0)
        for _ in range(2000):
            c0 = rng.uniform(-5, 5, 2)
            r0, r1 = rng.uniform(0.1, 6, 2)
            ang = rng.uniform(0, 2 * np.pi)
            for dist in (r0 + r1, abs(r0 - r1)):
                c1 = c0 + dist * np.array([np.cos(ang), np.sin(ang)])
                ints = circle_circle_intersection(c0, r0, c1, r1)
                self.assertEqual(len(ints), 2)
                self._assert_on_both_circles(ints, c0, r0, c1, r1, atol=1e-6)
                np.testing.assert_allclose(ints[0], ints[1], atol=1e-3)

    def test_circ_circ_intersection_random(self):
        """Random pairs: solution count matches geometry, points lie on both."""
        rng = np.random.default_rng(1)
        for _ in range(2000):
            c0, c1 = rng.uniform(-5, 5, 2), rng.uniform(-5, 5, 2)
            r0, r1 = rng.uniform(0.1, 6, 2)
            d = np.linalg.norm(c1 - c0)
            if abs(d - (r0 + r1)) < 1e-6 or abs(d - abs(r0 - r1)) < 1e-6:
                continue  # tangency handled by the dedicated test
            ints = circle_circle_intersection(c0, r0, c1, r1)
            expected = 0 if (d > r0 + r1 or d < abs(r0 - r1)) else 2
            self.assertEqual(len(ints), expected)
            self._assert_on_both_circles(ints, c0, r0, c1, r1)
            if ints:
                # the two points are mirror images across the line of centers
                u = (c1 - c0) / d
                m = (ints[0] + ints[1]) / 2 - c0
                self.assertAlmostEqual(u[0] * m[1] - u[1] * m[0], 0.0, delta=1e-9)

    def test_ik_xyz(self):
        np.random.seed(0)
        r = Rv6s(port=None)

        def fk_5th_joint_pos(q):
            t = np.eye(4)
            t[2, 3] = -r.dh_d[-1]
            return (r.fk(q) @ t)[:3, 3]

        for _ in range(100):
            q = np.random.uniform(r.q_min, r.q_max)
            exp_pos = fk_5th_joint_pos(q)
            sols = r._ik_5th_joint_pos(exp_pos)
            self.assertTrue(any([np.allclose(q[:3], s, atol=1e-6) for s in sols]))
            for s in sols:
                q[:3] = s
                np.testing.assert_allclose(fk_5th_joint_pos(q), exp_pos, atol=1e-6)

    def test_ik(self):
        np.random.seed(0)
        r = Rv6s(port=None)
        for i in range(100):
            q = np.random.uniform(r.q_min, r.q_max)
            pose = r.fk(q)
            sols = r.ik(pose)
            self.assertTrue(any([np.allclose(q, s, atol=1e-6) for s in sols]))
            for c in sols:
                np.testing.assert_allclose(r.fk(c), pose, atol=1e-6)


if __name__ == "__main__":
    unittest.main()
