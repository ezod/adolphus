"""\
Tensor module. Contains Tensor classes and a few operations such as a distance function
between tensors. The base Tensor class is implemented as an m x n matrix.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

from array import array
from cpython cimport bool

cdef class Tensor:
    """\
    Tensor class.
    """
    def __cinit__(self, matrix=[]):
        """\
        Constructor.
        
        @param matrix: The n x n tensor matrix.
        @type matrix: C{list} of C{list}
        """
        self._matrix = matrix
        self._h = len(matrix)
        if self._h == 0:
            self._w = 0
            vector = []
        else:
            try:
                self._w = len(matrix[0])
                if self._w == 0:
                    self._h = 0
                    vector = []
                else:
                    vector = []
                    for row in matrix:
                        assert len(row) == self._w, "All rows must have the same size."
                        for col in row:
                            assert isinstance(col, float) or isinstance(col, int), \
                                "No valid type cast exists from " + \
                                type(col).__name__ + " to \'float\'."
                            vector.append(col)
            except TypeError:
                self._w = 1
                for col in matrix:
                    assert isinstance(col, float) or isinstance(col, int), \
                        "No valid type cast exists from " + \
                        type(col).__name__ + " to \'float\'."
                vector = matrix
        assert self._h == self._w, "Tensor matrix must be square."
        self._tensor = array('d', vector)

    def __hash__(self):
        return hash(repr(self))
        
    def __reduce__(self):
        return (Tensor, (self._matrix))
        
    def __getitem__(self, val):
        if type(val).__name__ == 'int' or type(val).__name__ == 'slice':
            raise IndexError("Two indices are required.")
        if type(val).__name__ == 'tuple':
            i, j = val
            if type(i).__name__ == 'int':
                if i < 0 and i >= -self._h:
                    irange = [i + self._h]
                elif i >= 0 and i < self._h:
                    irange = [i]
                else:
                    raise IndexError("Index out of range.")
            elif type(i).__name__ == 'slice':
                if i.start == None:
                    i1 = 0
                elif i.start < 0 and i.start >= -self._h:
                    i1 = i.start + self._h
                elif i.start >= 0 and i.start < self._h:
                    i1 = i.start
                else:
                    raise IndexError("Index out of range.")
                if i.stop == None:
                    i2 = -1 + self._h
                elif i.stop < 0 and i.stop >= -self._h:
                    i2 = i.stop + self._h
                elif i.stop >= 0 and i.stop < self._h:
                    i2 = i.stop
                else:
                    raise IndexError("Index out of range.")
                irange = [item for item in xrange(i1, i2 + 1)]
            if type(j).__name__ == 'int':
                if j < 0 and j >= -self._w:
                    jrange = [j + self._w]
                elif j >= 0 and j < self._w:
                    jrange = [j]
                else:
                    raise IndexError("Index out of range.")
            elif type(j).__name__ == 'slice':
                if j.start == None:
                    j1 = 0
                elif j.start < 0 and j.start >= -self._w:
                    j1 = j.start + self._w
                elif j.start >= 0 and j.start < self._w:
                    j1 = j.start
                else:
                    raise IndexError("Index out of range.")
                if j.stop == None:
                    j2 = -1 + self._w
                elif j.stop < 0 and j.stop >= -self._w:
                    j2 = j.stop + self._w
                elif j.stop >= 0 and j.stop < self._w:
                    j2 = j.stop
                else:
                    raise IndexError("Index out of range.")
                jrange = [item for item in xrange(j1, j2 + 1)]
        out = []
        for i in irange:
            row = []
            for j in jrange:
                row.append(self._tensor.__getitem__((i * self._w) + j))
            out.append(row)
        return out

    def __richcmp__(self, Tensor t, int o):
        h1, w1 = self.size
        h2, w2 = t.size
        assert h1 == h2 and w1 == w2, "Both tensors must be of the same size."
        cdef bool eq = True
        for i in xrange(h1):
            for j in xrange(w2):
                if abs(self[i,j] - t[i,j]) > 1e-9:
                    eq = False
                    break
        if o == 2:
            return eq
        if o == 3:
            return not eq

    def __repr__(self):
        """\
        Canonical string representation.
        """
        line = type(self).__name__ + "(["
        for i in xrange(self._h):
            line += "["
            for j in xrange(self._w):
                if j < self._h - 1:
                    line += str(self._tensor.__getitem__((i * self._w) + j)) + ", "
                elif j == self._h - 1:
                    line += str(self._tensor.__getitem__((i * self._w) + j))
            if i < self._w - 1:
                line += "],\n        "
            elif i == self._w - 1:
                line += "]"
        line += "])"
        return line

    def __str__(self):
        """\
        String representation, displays in a tuple format.
        """
        line = "(["
        for i in xrange(self._h):
            line += "["
            for j in xrange(self._w):
                if j < self._h - 1:
                    line += str(self._tensor.__getitem__((i * self._w) + j)) + ", "
                elif j == self._h - 1:
                    line += str(self._tensor.__getitem__((i * self._w) + j))
            if i < self._w - 1:
                line += "],\n  "
            elif i == self._w - 1:
                line += "]"
        line += "])"
        return line

    @property
    def size(self):
        """\
        Return a tuple containing the height and width of the Tensor.
        """
        return (self._h, self._w)
