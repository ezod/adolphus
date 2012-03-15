#!/usr/bin/env python

"""\
Unit tests for the various Adolphus modules.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import unittest
from math import sqrt, pi, sin, cos

import adolphus
from adolphus.geometry import Angle, Point, DirectionalPoint, Pose, Rotation, Triangle
from adolphus.yamlparser import YAMLParser
print('Adolphus imported from "%s"' % adolphus.__path__[0])


class TestGeometry(unittest.TestCase):
    """\
    Tests for the geometry module.
    """
    def setUp(self):
        self.p = Point(3, 4, 5)
        self.dp = DirectionalPoint(-7, 1, 9, 1.3, 0.2)
        self.R = Rotation.from_euler('zyx', (pi, 0, 0))
        self.P1 = Pose(R=self.R)
        self.P2 = Pose(T=Point(3, 2, 1), R=self.R)

    def test_angle(self):
        a = Angle(0.3)
        self.assertTrue(abs(a - Angle(0.3 + 2 * pi)) < 1e-4)
        b = a + Angle(6.0)
        self.assertTrue(b < a)
        b = -a
        self.assertTrue(b > a)

    def test_point_eq(self):
        e = 9e-5
        self.assertEqual(self.p, Point(3 + e, 4 - e, 5 + e))

    def test_point_add_sub(self):
        self.assertEqual(self.p + self.dp, Point(-4, 5, 14))
        self.assertEqual(self.dp + self.p, DirectionalPoint(-4, 5, 14, 1.3, 0.2))
        self.assertEqual(self.p - self.dp, Point(10, 3, -4))

    def test_point_mul_div(self):
        self.assertEqual(self.p * 1.5, Point(4.5, 6, 7.5))
        self.assertEqual(self.p / 2, Point(1.5, 2, 2.5))

    def test_point_dot(self):
        self.assertEqual(self.p.dot(Point(2, 1, 3)), 25)

    def test_point_cross(self):
        self.assertEqual(self.p.cross(Point(1, 2, 1)), Point(-6, 2, 2))

    def test_point_neg(self):
        self.assertEqual(-self.p, Point(-3, -4, -5))
        self.assertEqual(-self.dp, DirectionalPoint(7, -1, -9, 1.3 + pi, 0.2))

    def test_point_magnitude(self):
        self.assertEqual(self.p.magnitude(), sqrt(sum([self.p[i] ** 2 for i in range(3)])))

    def test_point_unit(self):
        m = self.p.magnitude()
        self.assertEqual(self.p.unit(), Point(*[self.p[i] / m for i in range(3)]))

    def test_point_euclidean(self):
        self.assertEqual(self.p.euclidean(Point(0, 0, 0)), self.p.magnitude())

    def test_point_angle(self):
        self.assertTrue(abs(float(self.p.angle(-self.p)) - pi) < 1e-4)

    def test_point_direction_unit(self):
        rho, eta = self.dp.rho, self.dp.eta
        self.assertEqual(self.dp.direction_unit(), Point(sin(rho) * cos(eta), sin(rho) * sin(eta), cos(rho)))

    def test_rotation_rotate_point(self):
        r = Point(3, -4, -5)
        self.assertEqual(r, self.R.rotate(self.p))
        r = DirectionalPoint(-7, -1, -9, pi - 1.3, 2 * pi - 0.2)
        self.assertEqual(r, self.P1.map(self.dp))

    def test_pose_map(self):
        m = Point(6, -2, -4)
        self.assertEqual(m, self.P2.map(self.p))
        m = DirectionalPoint(-4, 1, -8, pi - 1.3, 2 * pi - 0.2)
        self.assertEqual(m, self.P2.map(self.dp))

    def test_triangle_intersection(self):
        triangle = Triangle(Point(-3, -3, 0), Point(-3, 2, 0), Point(4, 1, 0))
        self.assertTrue(triangle.intersection(Point(-1, -1, 3), Point(-1, -1, -3), True))
        self.assertTrue(triangle.intersection(Point(-1, -1, -3), Point(-1, -1, 3), True))
        self.assertFalse(triangle.intersection(Point(5, 5, 3), Point(5, 5, -3), True))
        self.assertFalse(triangle.intersection(Point(5, 5, 3), Point(5, 5, 1), True))

    def test_triangle_overlap(self):
        triangles = [Triangle(Point(0, 0, 0), Point(10, 2, 0), Point(8, 0, 6)),
                     Triangle(Point(0, 2, 1), Point(4, -7, 2), Point(7, 3, 3)),
                     Triangle(Point(-1, -1, -1), Point(-1, -2, 2), Point(-5, -1, -1))]
        self.assertTrue(triangles[0].overlap(triangles[1]))
        self.assertTrue(triangles[1].overlap(triangles[0]))
        self.assertFalse(triangles[0].overlap(triangles[2]))
        self.assertFalse(triangles[2].overlap(triangles[0]))
        self.assertFalse(triangles[1].overlap(triangles[2]))
        self.assertFalse(triangles[2].overlap(triangles[1]))


class TestModel01(unittest.TestCase):
    """\
    Test model 01.
    """
    def setUp(self):
        self.model, self.tasks = YAMLParser('test/test01.yaml').experiment

    def test_strength(self):
        p1 = Point(0, 0, 1000)
        p2 = Point(0, 0, 1200)
        self.assertEqual(self.model.strength(p1, self.tasks['R1'].params), self.model['C'].strength(p1, self.tasks['R1'].params))
        self.assertTrue(self.model.strength(p1, self.tasks['R1'].params))
        self.assertFalse(self.model.strength(p2, self.tasks['R1'].params))
        self.model['C'].set_absolute_pose(Pose(T=Point(1000, 0, 0)))
        self.assertFalse(self.model.strength(p1, self.tasks['R1'].params))
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(pi, Point(1, 0, 0))))
        self.assertFalse(self.model.strength(p1, self.tasks['R1'].params))

    def test_performance(self):
        self.assertTrue(self.model.performance(self.tasks['R1']) > 0)
        self.assertEqual(self.model.performance(self.tasks['R2']), 0.0)

    def test_occlusion_cache(self):
        key = self.model._update_occlusion_cache(self.tasks['R1'].params)
        self.assertTrue(all([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P1'].triangles]))
        self.assertFalse(any([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P2'].triangles]))
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(-pi / 2.0, Point(1, 0, 0))))
        key = self.model._update_occlusion_cache(self.tasks['R1'].params)
        self.assertFalse(any([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P1'].triangles]))
        self.assertTrue(all([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P2'].triangles]))
        self.model['C'].set_absolute_pose(Pose(R=Rotation.from_axis_angle(pi, Point(1, 0, 0))))
        key = self.model._update_occlusion_cache(self.tasks['R1'].params)
        self.assertFalse(any([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P1'].triangles]))
        self.assertFalse(any([t.mapped_triangle() in self.model._occlusion_cache[key]['C'].values() for t in self.model['P2'].triangles]))


if __name__ == '__main__':
    unittest.main()
