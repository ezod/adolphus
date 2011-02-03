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

    def test_angle(self):
        a = A.geometry.Angle(0.3)
        self.assertTrue(abs(a - A.geometry.Angle(0.3 + 2 * pi)) < 1e04)
        b = a + A.geometry.Angle(6.0)
        self.assertTrue(b < a)

    def test_point_addition(self):
        r = A.geometry.Point((-4, 5, 14))
        self.assertEqual(self.p + self.dp, r)
        s = A.geometry.DirectionalPoint((-4, 5, 14, 1.3, 0.2))
        self.assertEqual(self.dp + self.p, s)

    def test_point_rotation(self):
        r = A.geometry.Point((3, -4, -5))
        self.assertEqual(r, self.R.rotate(self.p))
        r = A.geometry.DirectionalPoint((-7, -1, -9, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(r, self.P1.map(self.dp))

    def test_pose_map(self):
        m = A.geometry.Point((6, -2, -4))
        self.assertEqual(m, self.P2.map(self.p))
        m = A.geometry.DirectionalPoint((-4, 1, -8, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(m, self.P2.map(self.dp))


class TestPlane(unittest.TestCase):
    """\
    Tests for plane intersections.
    """
    def setUp(self):
        self.pa = A.geometry.Point((0, 0, -10))
        self.pb = A.geometry.Point((0, 0, 10))
        self.pc = A.geometry.Point((0, 0, 20))
        self.corners = [A.geometry.Point((-10, -10, 0)),
                        A.geometry.Point((-10, 10, 0)),
                        A.geometry.Point((10, -10, 0)),
                        A.geometry.Point((10, 10, 0))]

    def test_orthogonal(self):
        plane = A.posable.Plane(x=[-10,10], y=[-10,10], z=0)
        self.assertEqual(plane.corners, self.corners)
        self.assertEqual(plane.intersection(self.pa, self.pb), A.geometry.Point())
        self.assertEqual(plane.intersection(self.pa, self.pc), A.geometry.Point())
        self.assertEqual(plane.intersection(self.pb, self.pc), None)

    def test_posed(self):
        pose = A.geometry.Pose(T=A.geometry.Point((12, -4, 5)),
            R=A.geometry.Rotation.from_euler('zyx', (0.3, 1.2, 0.1)))
        plane = A.posable.Plane(pose=pose, x=[-10,10], y=[-10,10])
        self.assertEqual(plane.corners, [pose.map(c) for c in self.corners])
        self.assertEqual(plane.intersection(pose.map(self.pa), pose.map(self.pb)), pose.map(A.geometry.Point()))
        self.assertEqual(plane.intersection(pose.map(self.pa), pose.map(self.pc)), pose.map(A.geometry.Point()))
        self.assertEqual(plane.intersection(pose.map(self.pb), pose.map(self.pc)), None)
        

if __name__ == '__main__':
    unittest.main()
