#!/usr/bin/env python

"""\
Unit tests for the various Adolphus modules.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import unittest

import adolphus as A
print "Adolphus imported from '%s'" % A.__path__[0]


class TestGeometry(unittest.TestCase):
    """\
    Tests for the geometry module.
    """
    def setUp(self):
        self.p = A.geometry.Point((3, 4, 5))
        self.q = A.geometry.DirectionalPoint((-7, 1, 9, 1.3, 0.2))

    def test_point_addition(self):
        r = A.geometry.Point((-4, 5, 14))
        self.assertEqual(self.p + self.q, r)
        s = A.geometry.DirectionalPoint((-4, 5, 14, 1.3, 0.2))
        self.assertEqual(self.q + self.p, s)

    def test_rotation_conversions(self):
        rotations = [A.geometry.Rotation(),
                     A.geometry.Rotation([0.3, 1.4, 2.1]),
                     A.geometry.Rotation([4.7, 0.9, 6.0]),
                     A.geometry.Rotation([[-0.601332, 0.67475, -0.427916],
                                          [0.797152, 0.543048, -0.263909],
                                          [0.054306, -0.499811, -0.86443]])]
        for R in rotations:
            converted = [A.geometry.Rotation(R.to_rotation_matrix()),
                         A.geometry.Rotation(R.to_axis_angle()),
                         A.geometry.Rotation(R.to_euler_xyz())]
            for C in converted:
                self.assertEqual(R.rotate(self.p), C.rotate(self.p))
                self.assertEqual(R.rotate(self.q), C.rotate(self.q))


if __name__ == '__main__':
    unittest.main()
