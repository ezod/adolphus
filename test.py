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

if __name__ == '__main__':
    unittest.main()
