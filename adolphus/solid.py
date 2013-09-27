"""\
Library with 3D model class. This library implements some manipulations on 3D models
that are represented as a triangular raw mesh.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""


from numpy import array
from collada import Collada, material, source, geometry, scene

from .geometry import Point, DirectionalPoint, Pose, Triangle, avg_points
from .posable import OcclusionTriangle, SceneObject


class Solid(SceneObject):
    """\
    Mesh polygon for 3D model representation class.
    """
    def __init__(self, file, name, pose=Pose(), mount_pose=Pose(), mount=None):
        """\
        Constructor.

        @param file: The file containing the object model.
        @type file: C{str}
        @param name: The name of the model.
        @type name: C{str}
        @param pose: Pose of the object in space (optional).
        @type pose: L{Pose}
        @param mount_pose: The transformation to the mounting end (optional).
        @type mount_pose: L{Pose}
        @param mount: Mount for this object (optional).
        @type mount: C{object}
        """
        self.solid_vertices = []
        self.solid_normals = []
        self.solid_indices = []
        self.solid_triangles = []
        self._single = []
        self._single_c = False
        self._raw = False
        self._dae = False
        if file[-4:] == '.raw':
            self._import_raw(file)
        elif file[-4:] == '.dae':
            self._import_dae(file)
        else:
            raise Exception("File format not supported.")

        occ_triangle = []
        for triangle in self.solid_triangles:
            occ_triangle.append(OcclusionTriangle(triangle.vertices, Pose(), None))
        SceneObject.__init__(self, name, pose=pose, mount_pose=mount_pose, \
            mount=mount, primitives=[], triangles=occ_triangle)
        self.visualize()

    def _import_raw(self, file):
        """\
        Import a raw triangulated mesh.
        """
        with open(file, 'r') as f:
            for line in f.readlines():
                line = line[:-2].split(' ')
                num = []
                for item in line:
                    num.append(float(item))
                points = [Point(num[0], num[1], num[2]), \
                          Point(num[3], num[4], num[5]), \
                          Point(num[6], num[7], num[8])]
                for point in points:
                    if point not in self.solid_vertices:
                        self.solid_vertices.append(point)
                self.solid_triangles.append(Triangle(*points))
        self._raw = True
        self._compute_normals()

    def _import_dae(self, file):
        """\
        Import a collada object.
        """
        solid = Collada(file)
        triangle_set = solid.geometries[0].primitives[0]
        for triangle in triangle_set:
            v = triangle.vertices
            points = [Point(v[0][0], v[0][1], v[0][2]), \
                      Point(v[1][0], v[1][1], v[1][2]), \
                      Point(v[2][0], v[2][1], v[2][2])]
            for point in points:
                    if point not in self.solid_vertices:
                        self.solid_vertices.append(point)
            self.solid_triangles.append(Triangle(*points))
        self._dae = True
        self._compute_normals()

    def _compute_normals(self):
        """\
        Compute the normals and the index list of this model.
        """
        n = 0
        for triangle in self.solid_triangles:
            self.solid_normals.append(triangle.normal())
            for vertex in triangle.vertices:
                self.solid_indices.append(self.solid_vertices.index(vertex))
                self.solid_indices.append(n)
            n += 1

    def _save_raw(self, name):
        """\
        Save the model as a raw file.
        """
        f = open(name+'.raw', 'w')
        for triangle in self.solid_triangles:
            line = ''
            for point in triangle.vertices:
                line += str(point.x) + ' ' + str(point.y) + ' ' + str(point.z) + ' '
            line = line[:-1]
            line += '\n'
            f.write(line)
        f.close()

    def _save_dae(self, name):
        """\
        Save the model as a collada file.
        """
        mesh = Collada()
        effect = material.Effect("effect0", [], "phong", diffuse=(0.9,0.9,0.9), \
            specular=(0.1,0.1,0.1))
        mat = material.Material("material0", "mymaterial", effect)
        mesh.effects.append(effect)
        mesh.materials.append(mat)
        vert_floats = []
        norm_floats = []
        for vertex in self.solid_vertices:
            vert_floats.append(vertex.x)
            vert_floats.append(vertex.y)
            vert_floats.append(vertex.z)
        for normal in self.solid_normals:
            norm_floats.append(normal.x)
            norm_floats.append(normal.y)
            norm_floats.append(normal.z)
        indices = array(self.solid_indices)
        vert_src = source.FloatSource("vert-array", array(vert_floats), \
            ('X', 'Y', 'Z'))
        norm_src = source.FloatSource("norm-array", array(norm_floats), \
            ('X', 'Y', 'Z'))
        geom = geometry.Geometry(mesh, "geometry0", "solid", [vert_src, norm_src])
        input_list = source.InputList()
        input_list.addInput(0, 'VERTEX', "#vert-array")
        input_list.addInput(1, 'NORMAL', "#norm-array")
        triset = geom.createTriangleSet(indices, input_list, "materialref")
        geom.primitives.append(triset)
        mesh.geometries.append(geom)
        matnode = scene.MaterialNode("materialref", mat, inputs=[])
        geomnode = scene.GeometryNode(geom, [matnode])
        node = scene.Node("node0", children=[geomnode])
        myscene = scene.Scene("myscene", [node])
        mesh.scenes.append(myscene)
        mesh.scene = myscene
        mesh.write(name+'.dae')

    def write(self, name='untitled', format='collada'):
        """\
        Save the model to a file of the specified format.

        @param name: The name of the file.
        @type name: C{str}
        @param format: The file format.
        @type format: C{str}
        """
        if '.' in name:
            name = name.split('.')[0]
        if format == 'collada':
            self._save_dae(name)
        elif format == 'raw':
            self._save_raw(name)
        else:
            raise Exception("File format not supported.")

    def scale(self, value):
        """\
        Scale the model by a factor on its x, y, and z coordinates.

        @param value: The scalar factor.
        @type value: C{float}
        """
        new_triangles = []
        for triangle in self.solid_triangles:
            points = []
            for point in triangle.vertices:
                points.append(point * value)
            new_triangles.append(Triangle(*points))
        del self.solid_triangles
        self.solid_triangles = new_triangles
        for i in range(len(self.solid_vertices)):
            self.solid_vertices[i] *= value
        # TODO: Update the occlusion triangle list of this SceneObject.
        self._compute_normals()
        self._single_c = False

    def single(self):
        """\
        Reduce the number of points per triangle to one. The resulting point is
        located at the centre of the triangle.

        @return: The list of points in the model.
        @rtype: C{list} of L{adolphus.geometry.DirectionalPoint}
        """
        if self._single_c:
            return self._single
        else:
            del self._single
            self._single = []
            for triangle in self.solid_triangles:
                average = avg_points(triangle.vertices)
                angles = triangle.normal_angles()
                point = DirectionalPoint(average.x, average.y, average.z, \
                    angles[0], angles[1])
                self._single.append(point)
            self._single_c = True
            return self._single
