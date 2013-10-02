"""\
Geometry module definition file.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from cpython cimport bool


cdef class Point:
    cdef public double x, y, z
    cdef double _magnitude
    cdef Point _unit
    cdef bool _magnitude_c, _unit_c
    cpdef Point _add(self, Point p)
    cpdef Point _sub(self, Point p)
    cpdef Point _mul(self, double s)
    cpdef Point _div(self, double s)
    cpdef Point _neg(self)
    cpdef double dot(self, Point p)
    cpdef Point cross(self, Point p)
    cpdef double cross_2d(self, Point p)
    cpdef double magnitude(self)
    cpdef Point unit(self)
    cpdef double euclidean(self, Point p)


cdef class DirectionalPoint(Point):
    cdef public object rho, eta
    cpdef Point direction_unit(self)


cdef class Quaternion:
    cdef public double a
    cdef public Point v
    cdef double _magnitude
    cdef Quaternion _unit, _conjugate, _inverse
    cdef bool _magnitude_c, _unit_c, _conjugate_c, _inverse_c
    cpdef Quaternion _add(self, Quaternion q)
    cpdef Quaternion _sub(self, Quaternion q)
    cpdef Quaternion _mul(self, Quaternion q)
    cpdef Quaternion _div(self, double s)
    cpdef Quaternion _neg(self)
    cpdef double magnitude(self)
    cpdef Quaternion unit(self)
    cpdef Quaternion conjugate(self)
    cpdef Quaternion inverse(self)


cdef class Rotation:
    cdef public Quaternion Q
    cpdef Rotation _add(self, Rotation other)
    cpdef Rotation inverse(self)
    cpdef Point rotate(self, Point p)


cdef class Pose:
    cdef readonly Point T
    cdef readonly Rotation R
    cdef Pose _inverse
    cdef bool _inverse_c
    cpdef Pose _add(self, Pose other)
    cpdef Pose inverse(self)
    cpdef Point _map(self, Point p)
    cpdef Point _dmap(self, Point p)
    cpdef Point map(self, Point p)


cdef class Face:
    cdef public object vertices
    cdef object _edges
    cdef Point _normal
    cdef Pose _planing_pose
    cdef bool _edges_c, _normal_c, _planing_pose_c
    cpdef object edges(self)
    cpdef Point normal(self)
    cpdef Pose planing_pose(self)
    cpdef double dist_to_point(self, Point p)
    cpdef object normal_angles(self)


cdef class Triangle(Face):
    cdef Point _vertex_0, _vertex_1, _vertex_2, _edge_0, _edge_1, _edge_2
    cpdef Triangle pose_map(self, Pose pose)
    cpdef Point intersection(self, Point origin, Point end, bool limit)
    cpdef bool overlap(self, Triangle other)
    cpdef bool is_inside(self, Point p)


cpdef bool point_in_segment(Point s1, Point s2, Point p)
cpdef bool segment_intersect(Point p1, Point p2, Point q1, Point q2)
cpdef bool triangle_frustum_intersection(Triangle triangle, object hull)
