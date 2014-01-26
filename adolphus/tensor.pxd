"""\
Tensor module declaration file.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

from cpython cimport bool


cdef class Tensor:
    cdef int _w, _h
    cdef object _m, _tensor
