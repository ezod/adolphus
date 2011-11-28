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

import adolphus
from adolphus.geometry import Angle, Point, DirectionalPoint, Pose, Rotation, Triangle
from adolphus.yamlparser import YAMLParser
print('Adolphus imported from "%s"' % adolphus.__path__[0])


class TestGeometry(unittest.TestCase):
    """\
    Tests for the geometry module.
    """
    def setUp(self):
        self.p = Point((3, 4, 5))
        self.dp = DirectionalPoint((-7, 1, 9, 1.3, 0.2))
        self.q = Rotation.from_euler('zyx', (pi, 0, 0))
        self.R = Rotation(self.q)
        self.P1 = Pose(T=Point(), R=self.R)
        self.P2 = Pose(T=Point((3, 2, 1)), R=self.R)

    def test_angle(self):
        a = Angle(0.3)
        self.assertTrue(abs(a - Angle(0.3 + 2 * pi)) < 1e04)
        b = a + Angle(6.0)
        self.assertTrue(b < a)

    def test_point_addition(self):
        r = Point((-4, 5, 14))
        self.assertEqual(self.p + self.dp, r)
        s = DirectionalPoint((-4, 5, 14, 1.3, 0.2))
        self.assertEqual(self.dp + self.p, s)

    def test_point_rotation(self):
        r = Point((3, -4, -5))
        self.assertEqual(r, self.R.rotate(self.p))
        r = DirectionalPoint((-7, -1, -9, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(r, self.P1.map(self.dp))

    def test_pose_map(self):
        m = Point((6, -2, -4))
        self.assertEqual(m, self.P2.map(self.p))
        m = DirectionalPoint((-4, 1, -8, pi - 1.3, 2 * pi - 0.2))
        self.assertEqual(m, self.P2.map(self.dp))

    def test_triangle_intersection(self):
        triangle = Triangle((Point((-3, -3, 0)), Point((-3, 2, 0)), Point((4, 1, 0))))
        self.assertTrue(triangle.intersection(Point((-1, -1, 3)), Point((-1, -1, -3))))
        self.assertTrue(triangle.intersection(Point((-1, -1, -3)), Point((-1, -1, 3))))
        self.assertFalse(triangle.intersection(Point((5, 5, 3)), Point((5, 5, -3))))
        self.assertFalse(triangle.intersection(Point((5, 5, 3)), Point((5, 5, 1))))


class TestModel01(unittest.TestCase):
    """\
    Test model 01.
    """
    def setUp(self):
        self.model, self.relevance_models = YAMLParser('test/test01.yaml').experiment

    def test_strength(self):
        p1 = Point((0, 0, 1000))
        p2 = Point((0, 0, 1200))
        self.assertEqual(self.model.strength(p1), self.model['C'].strength(p1))
        self.assertTrue(self.model.strength(p1))
        self.assertFalse(self.model.strength(p2))
        self.model['C'].set_absolute_pose(Pose(T=Point((1000, 0, 0))))
        self.assertFalse(self.model.strength(p1))
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(pi, Point((1, 0, 0)))))
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
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(-pi / 2.0, Point((1, 0, 0)))))
        self.model._update_occlusion_cache()
        self.assertFalse(self.model._occlusion_cache['C']['P1a'])
        self.assertFalse(self.model._occlusion_cache['C']['P1b'])
        self.assertTrue(self.model._occlusion_cache['C']['P2a'])
        self.assertTrue(self.model._occlusion_cache['C']['P2b'])
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(pi, Point((1, 0, 0)))))
        self.model._update_occlusion_cache()
        self.assertFalse(self.model._occlusion_cache['C']['P1a'])
        self.assertFalse(self.model._occlusion_cache['C']['P1b'])
        self.assertFalse(self.model._occlusion_cache['C']['P2a'])
        self.assertFalse(self.model._occlusion_cache['C']['P2b'])


if __name__ == '__main__':
    unittest.main()
