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
from numbers import Number

from .coverage import Camera
from .visualization import VISUAL_ENABLED
from .posable import OcclusionTriangle, SceneObject
from .geometry import Point, Rotation, Pose, point_segment_dis, avg_points

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
            single_i = False
            single_j = False
            i, j = val
            if type(i).__name__ == 'int':
                single_i = True
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
                irange = [item for item in range(i1, i2 + 1)]
            if type(j).__name__ == 'int':
                single_j = True
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
                jrange = [item for item in range(j1, j2 + 1)]
        if single_i and single_j:
            return self._tensor.__getitem__((i * self._w) + j)
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
        for i in range(h1):
            for j in range(w1):
                if abs(self[i,j] - t[i,j]) > 1e-9:
                    eq = False
                    break
        if o == 2:
            return eq
        if o == 3:
            return not eq

    def __neg__(self):
        """\
        Negate this tensor, except for the camera's 'y' axis, since it is assumed
        that rotations around the optical axis have no effect.
        Note: This function is meant for tensors of size 3x3.
    
        @return: Negated tensor.
        @rtype: L{Tensor}
        """
        n_matrix = [[-self[0,0], -self[0,1], self[0,2]], \
                    [-self[1,0], -self[1,1], self[1,2]], \
                    [-self[2,0], -self[2,1], self[2,2]]]
        return Tensor(n_matrix)

    def __repr__(self):
        """\
        Canonical string representation.
        """
        line = type(self).__name__ + "(["
        for i in range(self._h):
            line += "["
            for j in range(self._w):
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
        for i in range(self._h):
            line += "["
            for j in range(self._w):
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

    def unit(self):
        """\
        Normalize this tensor.
        Note: This function is meant for tensors of size 3x3.
        
        @return: Normalized tensor.
        @rtype: L{Tensor}
        """
        axis1 = Point(self[0,0], self[1,0], self[2,0]).unit()
        axis2 = Point(self[0,1], self[1,1], self[2,1]).unit()
        axis3 = Point(self[0,2], self[1,2], self[2,2]).unit()
        matrix = [[axis1.x, axis2.x, axis3.x], \
                  [axis1.y, axis2.y, axis3.y], \
                  [axis1.z, axis2.z, axis3.z]]
        return Tensor(matrix)

    def schatten(self):
        """\
        The schatten norm of this tensor.
        
        @return: The schatten norm.
        @rtype: L{float}
        """
        lambda1 = Point(self[0,0], self[1,0], self[2,0]).magnitude()
        lambda2 = Point(self[0,1], self[1,1], self[2,1]).magnitude()
        lambda3 = Point(self[0,2], self[1,2], self[2,2]).magnitude()
        return sqrt(lambda1**2 + lambda2**2 + lambda3**2)

    def frobenius(self, t):
        """\
        Compute the Frobenius distance from this tensor to another.
        
        @param t: The other tensor.
        @type t: L{Tensor}
        @return: Frobenius based distance.
        @rtype: C{float}
        """
        h1, w1 = self.size
        h2, w2 = t.size
        assert h1 == h2 and w1 == w2, "Both tensors must be of the same size."
        distance = 0
        for i in range(h1):
            for j in range(w1):
                distance += (t[i,j] - self[i,j]) ** 2
        return sqrt(distance)


class CameraTensor(Camera, Tensor):
    """\
    Camera Tensor class.
    """
    def __init__(self, task_params, name, params, pose=Pose(), mount_pose=Pose(), \
                 mount=None, primitives=list(), triangles=list()):
        """\
        Constructor.
        
        @param task_params: The task parameters.
        @type task_params: C{dict}
        @param name: The name of the camera.
        @type name: C{str}
        @param params: The intrinsic camera parameters.
        @type params: C{dict}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: Mount object for the camera (optional).
        @type mount: C{object}
        @param primitives: Sprite primitives for the camera.
        @type primitives: C{dict}
        @param triangles: The opaque triangles of this camera (optional).
        @type triangles: C{list} of L{OcclusionTriangle}
        """
        self._params = params
        self.task_params = task_params
        Camera.__init__(self, name, params, pose, mount_pose, mount, \
                        primitives, triangles)
        tensor_matrix = self._get_tensor_matrix(task_params)
        Tensor.__init__(self, tensor_matrix)
        if VISUAL_ENABLED:
            self.visualize()

    def setparam(self, param, value):
        """\
        Set a camera parameter.

        @param param: The name of the parameter to set.
        @type param: C{str}
        @param value: The value to which to set the parameter.
        @type value: C{float} or C{list} of C{float}
        """
        Camera.setparam(self, param, value)
        tensor_matrix = self._get_tensor_matrix(self.task_params)
        Tensor.__init__(self, tensor_matrix)

    def set_absolute_pose(self, value):
        Camera.set_absolute_pose(self, value)
        tensor_matrix = self._get_tensor_matrix(self.task_params)
        Tensor.__init__(self, tensor_matrix)

    absolute_pose = property(Camera.get_absolute_pose, set_absolute_pose)
    pose = absolute_pose

    def set_relative_pose(self, value):
        Camera.set_relative_pose(self, value)
        tensor_matrix = self._get_tensor_matrix(self.task_params)
        Tensor.__init__(self, tensor_matrix)

    relative_pose = property(Camera.get_relative_pose, set_relative_pose)

    def set_mount(self, value):
        Camera.set_mount(self, value)
        tensor_matrix = self._get_tensor_matrix(self.task_params)
        Tensor.__init__(self, tensor_matrix)

    mount = property(Camera.get_mount, set_mount)

    def _get_tensor_matrix(self, task_params):
        """\
        Compute the orthogonal basis of this camera tensor.
        
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: The tensor matrix.
        @rtype: C{list} of C{list}
        """
        hull = self.gen_frustum_hull(task_params)
        assert hull, "Camera has no coverage, unable to initialize Tensor."
        for i in range(len(hull)):
            hull[i] = Point(*hull[i])
        centre_c = avg_points(hull)
        first_axis = avg_points(hull[4:]) - centre_c
        second_axis = avg_points([hull[2], hull[3], hull[6], hull[7]]) - centre_c
        third_axis = avg_points([hull[0], hull[3], hull[4], hull[7]]) - centre_c
        # Store the coordinates of this tensor in the wcs.
        self._frustum_centre = self.pose.map(centre_c)
        # The Tensor is expressed in wcs.
        axis1 = self.pose.R.rotate(first_axis)
        axis2 = self.pose.R.rotate(second_axis)
        axis3 = self.pose.R.rotate(third_axis)
        return [[axis1.x, axis2.x, axis3.x], \
                [axis1.y, axis2.y, axis3.y], \
                [axis1.z, axis2.z, axis3.z]]

    def frustum_primitives(self, task_params):
        """\
        Generate the curve primitives for this camera's frustum for a given
        task.
    
        @param task_params: Task parameters.
        @type task_params: C{dict}
        @return: Frustum primitives.
        @rtype: C{list} of C{dict}
        """
        hull = self.gen_frustum_hull(task_params)
        if not hull:
            return []
        # Return the primitives based on the hull.
        primitives = [{'type': 'curve', 'color': (1, 0, 0), 'pos': hull[i:i + 4] + \
            hull[i:i + 1]} for i in range(0, 16, 4)] + \
            [{'type': 'curve', 'color': (1, 0, 0),
            'pos': [hull[i], hull[i + 4]]} for i in range(4)]
        # Include the orthogonal basis for the Camera Tensor.
        for i in range(len(hull)):
            hull[i] = Point(*hull[i])
        centre_c = avg_points(hull)
        first_axis = avg_points(hull[4:]) - centre_c
        second_axis = avg_points([hull[2], hull[3], hull[6], hull[7]]) - centre_c
        third_axis = avg_points([hull[0], hull[3], hull[4], hull[7]]) - centre_c
        for axis in [first_axis, second_axis, third_axis]:
            primitives.append({'type':          'arrow',
                               'pos':           centre_c.to_list(),
                               'axis':          axis.to_list(),
                               'shaftwidth':    3,
                               'color':         [0, 0, 1],
                               'material':      visual.materials.emissive})
        return primitives

    @property
    def centre(self):
        """\
        Return the location in the wcs of this camera's tensor.
        """
        return self._frustum_centre
    
    @property
    def axis(self):
        """\
        Return the coordinates of this camera's optical axis in the camera's frame.
        """
        return Point(self[0,0], self[1,0], self[2,0])

    def weighted_euclidean(self, other):
        """\
        Compute the euclidean distance from this tensor to another. The distance is
        weighted by the degree of coloneairty between the principal axes of this and
        the other tensor.
        
        @param other: The other tensor.
        @type other: L{CameraTensor} or L{TriangleTensor}
        @return: The distance.
        @rtype: C{float}
        """
        centre_dis = self.centre.euclidean(other.centre)
        collinear = self.axis.unit().dot(-other.axis.unit())
        if collinear > 0:
            return (1 / collinear) * centre_dis
        return -1

    def weighted_frobenius(self, other):
        """
        Compute the Frobenius distance from this tensor to another. The distance is
        weighted by the euclidean distance between the tensors.
        
        @param other: The other tensor.
        @type other: L{CameraTensor} or L{TriangleTensor}
        @return: The distance.
        @rtype: C{float}
        """
        euc_dist = self.centre.euclidean(other.centre)
        frob_dis = self.unit().frobenius(-other.unit())
        try:
            return (1 / (1 - (frob_dis / sqrt(8)))) * euc_dist
        except ZeroDivisionError:
            return -1

    def strength(self, triangle):
        """\
        Return the equivalent to the coverage strength for a triangle tensor.
    
        @param point: The triangle to test.
        @type point: L{TriangleTensor}
        @return: The vision distance scaled to the limits of the frustum.
        @rtype: C{float}
        """
        vd = self.weighted_frobenius(triangle)
        ratio = vd / self.schatten()
        if ratio > 1:
            return 0
        return 1 - ratio


class TriangleTensor(OcclusionTriangle, Tensor):
    """\
    Triangle Tensor class.
    """
    def __init__(self, vertices, pose=Pose(), mount=None):
        """\
        Constructor.

        @param vertices: The vertices of the triangle.
        @type vertices: C{tuple} of L{Point}
        @param pose: The pose of the triangle normal (from z-hat).
        @type pose: L{Pose}
        @param mount: The mount of the triangle (optional).
        @type mount: L{adolphus.posable.Posable}
        """
        self._guide_c = False
        OcclusionTriangle.__init__(self, vertices, pose, mount)
        tensor_matrix = self._get_tensor_matrix(self.triangle.vertices)
        Tensor.__init__(self, tensor_matrix)
        if VISUAL_ENABLED:
            self.visualize()

    def set_absolute_pose(self, value):
        OcclusionTriangle.set_absolute_pose(self, value)
        tensor_matrix = self._get_tensor_matrix(self.triangle.vertices)
        Tensor.__init__(self, tensor_matrix)
        if self._guide_c:
            self.toggle_tensor_vis()
            self.toggle_tensor_vis()
    
    absolute_pose = property(OcclusionTriangle.get_absolute_pose, set_absolute_pose)
    pose = absolute_pose
    
    def set_relative_pose(self, value):
        OcclusionTriangle.set_relative_pose(self, value)
        tensor_matrix = self._get_tensor_matrix(self.triangle.vertices)
        Tensor.__init__(self, tensor_matrix)
        if self._guide_c:
            self.toggle_tensor_vis()
            self.toggle_tensor_vis()
    
    relative_pose = property(OcclusionTriangle.get_relative_pose, set_relative_pose)
    
    def set_mount(self, value):
        OcclusionTriangle.set_mount(self, value)
        tensor_matrix = self._get_tensor_matrix(self.triangle.vertices)
        Tensor.__init__(self, tensor_matrix)
        if self._guide_c:
            self.toggle_tensor_vis()
            self.toggle_tensor_vis()
    
    mount = property(OcclusionTriangle.get_mount, set_mount)

    def _pose_changed_hook(self):
        """\
        Hook called on pose change.
        """
        OcclusionTriangle._pose_changed_hook(self)
        tensor_matrix = self._get_tensor_matrix(self.triangle.vertices)
        Tensor.__init__(self, tensor_matrix)
        if self._guide_c:
            self.toggle_tensor_vis()
            self.toggle_tensor_vis()

    def _get_tensor_basis(self, vertices):
        """\
        Compute the orthogonal basis of this triangle tensor (in the triangle's
        frame).
        
        @param vertices: The vertices of the triangle.
        @type vertices: C{tuple} of L{Point}
        @return: The tensor matrix.
        @rtype: C{list} of C{list}
        """
        centre = avg_points(vertices)
        vector1 = vertices[0] - centre
        vector2 = vertices[1] - centre
        vector3 = vertices[2] - centre
        # The magnitude of the axis should be the same as the radius of the largest
        # circle that can grow from the centre of the triangle without crossing the
        # edges of the triangle.
        mag1 = point_segment_dis(vertices[0], vertices[1], centre)
        mag2 = point_segment_dis(vertices[0], vertices[2], centre)
        mag3 = point_segment_dis(vertices[1], vertices[2], centre)
        mag = min([mag1, mag2, mag3])
        # Choose the most coolinear pair of vectors (from the centre to the vertices)
        # as the cross product that will result on this tensor's principal axis.
        if abs(vector1.dot(vector2)) > abs(vector1.dot(vector3)):
            pair = [vector1, vector2]
        else:
            pair = [vector1, vector3]
        if abs(pair[0].dot(pair[1])) < abs(vector2.dot(vector3)):
            pair = [vector2, vector3]
        first_axis = pair[0].unit().cross(pair[1].unit())
        second_axis = pair[0].unit()
        third_axis = second_axis.cross(first_axis)
        return [first_axis * mag, second_axis * mag, third_axis * mag]

    def _get_tensor_matrix(self, vertices):
        """\
        Compute the orthogonal basis of this triangle tensor (in wcs).
        
        @param vertices: The vertices of the triangle.
        @type vertices: C{tuple} of L{Point}
        @return: The tensor matrix.
        @rtype: C{list} of C{list}
        """
        centre = avg_points(vertices)
        basis = self._get_tensor_basis(vertices)
        planing_pose = self.triangle.planing_pose()
        pose = self.pose - planing_pose
        # Store the coordinates of this tensor in the wcs.
        self._centre = pose.map(centre)
        # The Tensor is expressed in wcs.
        axis1 = pose.R.rotate(basis[0])
        axis2 = pose.R.rotate(basis[1])
        axis3 = pose.R.rotate(basis[2])
        return [[axis1.x, axis2.x, axis3.x], \
                [axis1.y, axis2.y, axis3.y], \
                [axis1.z, axis2.z, axis3.z]]

    @property
    def centre(self):
        """\
        Return the location in the wcs of this triangle's tensor.
        """
        return self._centre
    
    @property
    def axis(self):
        """\
        Return the coordinates of this triangle's surface normal in the triangle's
        frame (with respecto to it's centre and not the fist vertex).
        """
        return Point(self[0,0], self[1,0], self[2,0])

    def toggle_tensor_vis(self):
        """\
        Toggle the visualization of this tensor's basis.
        """
        if self._guide_c:
            self.guide.visible = False
            del self.guide
            self._guide_c = False
        else:
            basis = self._get_tensor_basis(self.triangle.vertices)
            centre = avg_points(self.triangle.vertices)
            primitives = []
            for axis in basis:
                primitives.append({'type':          'arrow',
                                   'pos':           centre.to_list(),
                                   'axis':          axis.to_list(),
                                   'shaftwidth':    3,
                                   'color':         [0, 0, 1],
                                   'material':      visual.materials.emissive})
            self.guide = SceneObject('guide', mount=self, primitives=primitives)
            self.guide.visualize()
            self._guide_c = True
