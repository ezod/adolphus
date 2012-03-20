"""\
Posable objects module definition file.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from cpython cimport bool
from geometry cimport Pose


cdef class Posable:
    cdef public object children, posecallbacks
    cdef public Pose _pose, _mount_pose, _absolute_pose
    cdef public Posable _mount
    cdef public bool _absolute_pose_c
    cpdef Pose get_absolute_pose(self)
    cpdef set_absolute_pose(self, Pose value)
    cpdef Pose get_relative_pose(self)
    cpdef set_relative_pose(self, Pose value)
    cpdef _pose_changed_hook(self)
    cpdef Posable get_mount(self)
    cpdef set_mount(self, Posable value)
