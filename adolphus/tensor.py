"""\
Tensor module. Contains Tensor classes and a few operations such as a distance function
between tensors. The base Tensor class is implemented as an m x n matrix.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

import numpy as np
from math import sqrt
from array import array

from adolphus.geometry import Point, avg_points


class Tensor(object):
    """\
    Tensor class.
    """
    def __init__(self, matrix=[]):
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
                        assert len(row) == self._w, "All rows must have the " \
                            + "same size."
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
        #assert self._h == self._w, "Tensor matrix must be square."
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
                    raise IndexError("Row index out of range.")
            elif type(i).__name__ == 'slice':
                if i.start == None:
                    i1 = 0
                elif i.start < 0 and i.start >= -self._h:
                    i1 = i.start + self._h
                elif i.start >= 0 and i.start < self._h:
                    i1 = i.start
                else:
                    raise IndexError("Row index out of range.")
                if i.stop == None:
                    i2 = -1 + self._h
                elif i.stop < 0 and i.stop >= -self._h:
                    i2 = i.stop + self._h
                elif i.stop >= 0 and i.stop < self._h:
                    i2 = i.stop
                else:
                    raise IndexError("Row index out of range.")
                irange = [item for item in xrange(i1, i2 + 1)]
            if type(j).__name__ == 'int':
                if j < 0 and j >= -self._w:
                    jrange = [j + self._w]
                elif j >= 0 and j < self._w:
                    jrange = [j]
                else:
                    raise IndexError("Column index out of range.")
            elif type(j).__name__ == 'slice':
                if j.start == None:
                    j1 = 0
                elif j.start < 0 and j.start >= -self._w:
                    j1 = j.start + self._w
                elif j.start >= 0 and j.start < self._w:
                    j1 = j.start
                else:
                    raise IndexError("Column index out of range.")
                if j.stop == None:
                    j2 = -1 + self._w
                elif j.stop < 0 and j.stop >= -self._w:
                    j2 = j.stop + self._w
                elif j.stop >= 0 and j.stop < self._w:
                    j2 = j.stop
                else:
                    raise IndexError("Column index out of range.")
                jrange = [item for item in xrange(j1, j2 + 1)]
        out = []
        for i in irange:
            row = []
            for j in jrange:
                row.append(self._tensor.__getitem__((i * self._w) + j))
            out.append(row)
        return out

    def __richcmp__(self, t, o):
        h1, w1 = self.size
        h2, w2 = t.size
        assert h1 == h2 and w1 == w2, "Both tensors must be of the same size."
        eq = True
        for i in xrange(h1):
            for j in xrange(w1):
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

    def frobenius(self, t):
        """\
        Compute the frobenius distance from this tensor to another.
        
        @param t: The other tensor.
        @type t: L{Tensor}
        @return: Frobenius based distance.
        @rtype: C{float}
        """
        h1, w1 = self.size
        h2, w2 = t.size
        assert h1 == h2 and w1 == w2, "Both tensors must be of the same size."
        distance = 0
        for i in xrange(h1):
            for j in xrange(w1):
                distance += (t[i,j] - self[i,j]) ** 2
        return sqrt(distance)


class CameraTensor(Tensor):
    """\
    Camera Tensor calss.
    """
    def __init__(self, camera, task_params):
        """\
        Constructor.
        
        @param camera: The camera to aproximate.
        @type camera: L{adolphus.coverage.Camera}
        @param task_params: The task parameters.
        @type task_params: C{dict}
        """
        # Find the minimum and maximum depths of coverage in the frustum.
        z_lim = [max(camera.zres(task_params['res_max'][1]),
                     camera.zc(task_params['blur_max'][1] * \
                     min(camera._params['s']))[0]),
                 min(camera.zres(task_params['res_min'][1]),
                     camera.zc(task_params['blur_max'][1] * \
                     min(camera._params['s']))[1])]
        # No primitives if no coverage.
        assert z_lim[0] < z_lim[1], "This camera has no coverage."
        # Otherwise, generate the hull.
        hull = []
        for z in z_lim:
            hull += [Point(camera.fov['tahl'] * z, camera.fov['tavt'] * z, z),
                     Point(camera.fov['tahl'] * z, camera.fov['tavb'] * z, z),
                     Point(camera.fov['tahr'] * z, camera.fov['tavb'] * z, z),
                     Point(camera.fov['tahr'] * z, camera.fov['tavt'] * z, z)]
        centre_c = avg_points(hull)
        first_axis = avg_points(hull[4:]) - centre_c
        second_axis = avg_points([hull[2], hull[3], hull[6], hull[7]]) - centre_c
        third_axis = avg_points([hull[0], hull[3], hull[4], hull[7]]) - centre_c
        matrix1 = np.array([[first_axis.x, second_axis.x, third_axis.x], \
                            [first_axis.y, second_axis.y, third_axis.y], \
                            [first_axis.z, second_axis.z, third_axis.z]])
        lambda1 = first_axis.magnitude()
        lambda2 = second_axis.magnitude()
        lambda3 = third_axis.magnitude()
        shape = np.array([[lambda1, 0, 0],[0, lambda2, 0],[0, 0, lambda3]])
        first_axis = first_axis.unit()
        second_axis = second_axis.unit()
        third_axis = third_axis.unit()
        orth_basis = np.array([[first_axis.x, second_axis.x, third_axis.x], \
                               [first_axis.y, second_axis.y, third_axis.y], \
                               [first_axis.z, second_axis.z, third_axis.z]])
        print np.linalg.eig(matrix1)
        print '\n'
        print shape
        print '\n'
        print orth_basis
        print '\n'
        print '\n'
        print '\n'
        print shape * orth_basis
        print '\n'
        print matrix1
        Tensor.__init__(self, matrix1)


class TiangleTensor(Tensor):
    """\
    Triangle Tensor class.
    """
    def __init__(self, triangle):
        """\
        Constructor.
        """
        # TODO: Implement.
        Tensor.__init__(self, [])
