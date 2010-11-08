#!/usr/bin/env python

"""\
Unit tests for the various Adolphus modules.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import unittest
from math import pi

import adolphus as A
print "Adolphus imported from '%s'" % A.__path__[0]


class TestGeometry(unittest.TestCase):
    """\
    Tests for the geometry module.
    """
    def setUp(self):
        self.p = A.geometry.Point((3, 4, 5))
        self.dp = A.geometry.DirectionalPoint((-7, 1, 9, 1.3, 0.2))
        self.q = A.geometry.Rotation.from_euler('zyx', (pi, 0, 0))
        self.R = A.geometry.Rotation(self.q)
        self.P1 = A.geometry.Pose(T=A.geometry.Point(), R=self.R)
        self.P2 = A.geometry.Pose(T=A.geometry.Point((3, 2, 1)), R=self.R)

    def test_point_addition(self):
        r = A.geometry.Point((-4, 5, 14))
        self.assertEqual(self.p + self.dp, r)
        s = A.geometry.DirectionalPoint((-4, 5, 14, 1.3, 0.2))
        self.assertEqual(self.dp + self.p, s)

    def test_point_rotation(self):
        r = A.geometry.Point((3, -4, -5))
        self.assertEqual(r, self.R.rotate(self.p))
        r = A.geometry.DirectionalPoint((-7, -1, -9, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(r, self.P1.map_rotate(self.dp))

    def test_pose_map(self):
        m = A.geometry.Point((6, -2, -4))
        self.assertEqual(m, self.P2.map(self.p))
        m = A.geometry.DirectionalPoint((-4, 1, -8, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(m, self.P2.map(self.dp))


if __name__ == '__main__':
    unittest.main()
