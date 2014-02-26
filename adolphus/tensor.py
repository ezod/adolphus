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

from .posable import SceneObject
from .visualization import VISUAL_ENABLED
from .geometry import Point, Rotation, Pose, avg_points

if VISUAL_ENABLED:
    import visual


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


class CameraTensor(Tensor, SceneObject):
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
        assert z_lim[0] < z_lim[1], "Camera has no coverage, unable to " + \
            "initialize Tensor."
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
        self.centre = camera.pose.map(centre_c)
        self.axis1 = camera.pose.R.rotate(first_axis)
        self.axis2 = camera.pose.R.rotate(second_axis)
        self.axis3 = camera.pose.R.rotate(third_axis)
        matrix1 = np.array([[self.axis1.x, self.axis2.x, self.axis3.x], \
                            [self.axis1.y, self.axis2.y, self.axis3.y], \
                            [self.axis1.z, self.axis2.z, self.axis3.z]])
        primitives = []
        if VISUAL_ENABLED:
            for axis in [self.axis1, self.axis2, self.axis3]:
                primitives.append({'type':          'arrow',
                                   'pos':           self.centre.to_list(),
                                   'axis':          axis.to_list(),
                                   'shaftwidth':    3,
                                   'color':         [0, 0, 1],
                                   'material':      visual.materials.emissive})
        Tensor.__init__(self, matrix1.tolist())
        SceneObject.__init__(self, camera.name + "Tensor", \
                             primitives=primitives, triangles=[])
        if VISUAL_ENABLED:
            self.visualize()


class TiangleTensor(Tensor):
    """\
    Triangle Tensor class.
    """
    def __init__(self, triangle):
        """\
        Constructor.
        """
        # TODO: Implement this class.
        Tensor.__init__(self, [])
