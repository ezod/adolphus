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
print('Adolphus imported from "%s"' % A.__path__[0])


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

    def test_triangle_intersection(self):
        triangle = A.Triangle((A.Point((-3, -3, 0)), A.Point((-3, 2, 0)), A.Point((4, 1, 0))))
        self.assertEqual(triangle.intersection(A.Point((-1, -1, 3)), A.Point((-1, -1, -3))), A.Point((-1, -1, 0)))
        self.assertEqual(triangle.intersection(A.Point((5, 5, 3)), A.Point((5, 5, -3))), None)
        self.assertEqual(triangle.intersection(A.Point((5, 5, 3)), A.Point((5, 5, 1))), None)


class TestModel01(unittest.TestCase):
    """\
    Test model 01.
    """
    def setUp(self):
        self.model, self.relevance_models = A.YAMLParser('test/test01.yaml').experiment

    def test_strength(self):
        p1 = A.Point((0, 0, 1000))
        p2 = A.Point((0, 0, 1200))
        self.assertEqual(self.model.strength(p1), self.model['C'].strength(p1))
        self.assertTrue(self.model.strength(p1))
        self.assertFalse(self.model.strength(p2))
        self.model['C'].set_absolute_pose(A.Pose(T=A.Point((1000, 0, 0))))
        self.assertFalse(self.model.strength(p1))
        self.model['C'].set_absolute_pose(A.Pose(R=A.Rotation.from_axis_angle(pi, A.Point((1, 0, 0)))))
        self.assertFalse(self.model.strength(p1))

    def test_performance(self):
        self.assertEqual(self.model.performance(self.relevance_models['R1']), 1.0)
        self.assertEqual(self.model.performance(self.relevance_models['R2']), 0.0)

    def test_occlusion_cache(self):
        self.model._update_occlusion_cache()
        self.assertTrue(self.model._occlusion_cache['C']['P1a'])
        self.assertTrue(self.model._occlusion_cache['C']['P1b'])
        self.assertFalse(self.model._occlusion_cache['C']['P2a'])
        self.assertFalse(self.model._occlusion_cache['C']['P2b'])
        self.model['C'].set_absolute_pose(A.Pose(R=A.Rotation.from_axis_angle(-pi / 2.0, A.Point((1, 0, 0)))))
        self.model._update_occlusion_cache()
        self.assertFalse(self.model._occlusion_cache['C']['P1a'])
        self.assertFalse(self.model._occlusion_cache['C']['P1b'])
        self.assertTrue(self.model._occlusion_cache['C']['P2a'])
        self.assertTrue(self.model._occlusion_cache['C']['P2b'])
        self.model['C'].set_absolute_pose(A.Pose(R=A.Rotation.from_axis_angle(pi, A.Point((1, 0, 0)))))
        self.model._update_occlusion_cache()
        self.assertFalse(self.model._occlusion_cache['C']['P1a'])
        self.assertFalse(self.model._occlusion_cache['C']['P1b'])
        self.assertFalse(self.model._occlusion_cache['C']['P2a'])
        self.assertFalse(self.model._occlusion_cache['C']['P2b'])


if __name__ == '__main__':
    unittest.main()
