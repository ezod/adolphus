"""\
Mesh polygon library for 3D model representation.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""


from numpy import array
from collada import Collada, material, source, geometry, scene

HYPERGRAPH_ENABLED = True
try:
    import hypergraph
except ImportError:
    HYPGERGRAPH_ENABLED = False

from .coverage import PointCache
from .visualization import VISUAL_ENABLED
from .posable import OcclusionTriangle, SceneObject
from .geometry import Point, DirectionalPoint, Pose, Triangle, avg_points


class RenderDynamic(object):
    """\
    Render dynamic class.
    """
    def __init__(self, triangles):
        """\
        Constructor.

        @param triangles: The list of triangles of the model.
        @type triangles: C{list} of L{adolphus.geometry.Triangle}
        """
        self._originals = triangles
        self.compute_topology()

    def compute_topology(self):
        """\
        Compute the normals and the index list of this model.
        """
        try:
            del self._graph
            del self.vertices
            del self.edges
            del self.faces
            del self.normals
            del self.collada_indices
        except:
            pass
        self.vertices = []
        self.edges = []
        self.faces = []
        self.normals = []
        self.collada_indices = []
        n = 0
        for item in self._originals:
            j = 2
            for i in [0,1,2]:
                _edge = (item.vertices[i], item.vertices[j])
                if _edge not in self.edges and \
                    (item.vertices[j], item.vertices[i]) not in self.edges:
                    self.edges.append(_edge)
                if item.vertices[i] not in self.vertices:
                    self.vertices.append(item.vertices[i])
                j = i
            self.normals.append(item.normal())
            for vertex in item.vertices:
                self.collada_indices.append(self.vertices.index(vertex))
                self.collada_indices.append(n)
            n += 1
        self.faces = self._originals

    def _build_graph(self):
        """\
        Build the topology graph of this object.
        """
        if not HYPERGRAPH_ENABLED:
            raise ImportError('Hypergraph module not loaded.')
        try:
            del self._graph
        except:
            pass
        self._graph = hypergraph.core.Graph(self.vertices)
        for edge in self.edges:
            self._graph.add_edge(hypergraph.core.Edge(edge))

    @property
    def graph(self):
        """\
        Return the topology graph of this object.
        """
        try:
            return self._graph
        except:
            self._build_graph()
            return self._graph

    def _vertex_vertex(self):
        """\
        Generate the vertex-vertex list.
        """
        try:
            del self.vertex_vertex
        except:
            pass
        self.vertex_vertex = []
        for vertex in self.vertices:
            neighbors = self.graph.neighbors(vertex)
            self.vertex_vertex.append([n for n in neighbors])

    def vertex_neighbors(self, vertex):
        """\
        Find the vertices around a vertex.

        @param vertex: The vertex.
        @type vertex: L{adolphus.geometry.Point}
        @return: The vertices around the given vertex.
        @rtype: C{list} of L{adolphus.geometry.Point}
        """
        index = self.vertices.index(vertex)
        try:
            return self.vertex_vertex[index]
        except:
            self._vertex_vertex()
            return self.vertex_neighbors(vertex)

    def _edge_face(self):
        """\
        Generate the edge-face list.
        """
        try:
            del self.edge_face
        except:
            pass
        self.edge_face = []
        for face in self.faces:
            j = 2
            edges = []
            for i in [0,1,2]:
                edges.append((face.vertices[i], face.vertices[j]))
                j = i
            self.edge_face.append(edges)

    def edges_of_face(self, face):
        """\
        Find all the edges of a face.

        @param face: The face.
        @type face: L{adolphus.geometry.Triangle}
        @return: The edges of the given face.
        @rtype: C{list} of C{tuple} of L{adolphus.geometry.Point}
        """
        index = self.faces.index(face)
        try:
            return self.edge_face[index]
        except:
            self._edge_face()
            return self.edges_of_face(face)

    def _face_vertex(self):
        """\
        Generate the face-vertex list.
        """
        try:
            del self.face_vertex
        except:
            pass
        self.face_vertex = []
        for vertex in self.vertices:
            faces = []
            for i in range(len(self.faces)):
                if vertex in self.faces[i].vertices:
                    faces.append(i)
            self.face_vertex.append(faces)

    def faces_of_vertex(self, vertex):
        """\
        Find the faces around a vertex.

        @param vertex: The vertex.
        @type vertex: L{adolphus.geometry.Point}
        @return: The indices of the faces around a vertex.
        @rtype: C{list} of C{int}
        """
        index = self.vertices.index(vertex)
        try:
            return self.face_vertex[index]
        except:
            self._face_vertex()
            return self.faces_of_vertex(vertex)

    def _edge_vertex(self):
        """\
        Generate the edge-vertex list.
        """
        try:
            del self.edge_vertex
        except:
            pass
        self.edge_vertex = []
        for vertex in self.vertices:
            self.edge_vertex.append([edge for edge in self.edges \
                if vertex in edge])

    def edges_of_vertex(self, vertex):
        """\
        Find the edges around a vertex.

        @param vertex: The vertex.
        @type vertex: L{adolphus.geometry.Point}
        @return: The edges around a vertex.
        @rtype: C{list} of C{tuple} of L{adolphus.geometry.Point}
        """
        index = self.vertices.index(vertex)
        try:
            return self.edge_vertex[index]
        except:
            self._edge_vertex()
            return self.edges_of_vertex(vertex)

    def _face_edge(self):
        """\
        Generate the face-edge list.
        """
        try:
            del self.face_edge
        except:
            pass
        if not hasattr(self, 'edge_face'):
            self._edge_face()
        self.face_edge = []
        for edge in self.edges:
            faces = []
            for i in range(len(self.edge_face)):
                if edge in self.edge_face[i] or \
                    (edge[1], edge[0]) in self.edge_face[i]:
                    faces.append(i)
            self.face_edge.append(faces)

    def faces_of_edge(self, edge):
        """\
        Find the faces of an edge.

        @param edge: The edge.
        @type edge: C{tuple} of L{adolphus.geometry.Point}
        @return: The indices of the faces of the edge.
        @rtype: C{list} of C{int}
        """
        index = self.edges.index(edge)
        try:
            return self.face_edge[index]
        except:
            self._face_edge()
            return self.faces_of_edge(edge)

    def flook(self, a, b, c):
        """\
        Find the face with the given vertices (in counterclockwise order
        looking at the face).

        @param a: The first vertex.
        @type a: L{adolphus.geometry.Point}
        @param b: The second vertex.
        @type b: L{adolphus.geometry.Point}
        @param c: The third vertex.
        @type c: L{adolphus.geometry.Point}
        @return: The index of the face.
        @rtype: C{int}
        """
        assert(a!=b and b!=c and a!=c), "Invalid triangle."
        for i in range(len(self.faces)):
            if a in self.faces[i].vertices and b in self.faces[i].vertices and \
                c in self.faces[i].vertices:
                return i

    def gen_render_dynamic(self):
        """\
        Compute all the lists that make up for the render dynamic data structure.
        """
        self._vertex_vertex()
        self._edge_face()
        self._face_vertex()
        self._edge_vertex()
        self._face_edge()

    def find_boundary(self):
        """\
        Find the boundary of this solid.

        @return: The edges in the boundary of this model.
        @rtype: C{list} of C{tuple} of L{adolphus.geometry.Point}
        """
        try:
            indices = [i for i in range(len(self.face_edge)) \
                if len(self.face_edge[i]) == 1]
            edges = [self.edges[i] for i in indices]
            return edges
        except:
            self.gen_render_dynamic()
            return self.find_boundary()

    def remove_face(self, indices):
        """\
        Remove a face or faces from this object.

        @param indices: The indices of the faces to remove.
        @type indices: C{list} of C{int}
        """
        faces = [self.faces[i] for i in indices]
        for face in faces:
            self.faces.pop(self.faces.index(face))
        self.compute_topology()
        self.gen_render_dynamic()


class Solid(RenderDynamic, SceneObject):
    """\
    Solid class.

    Refer to http://en.wikipedia.org/wiki/Polygon_mesh for a description
    of render dynamic.
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
        self._single = PointCache()
        self._single_c = False
        if file[-4:] == '.raw':
            self._import_raw(file)
        elif file[-4:] == '.dae':
            self._import_dae(file)
        else:
            raise Exception("File format not supported.")

        RenderDynamic.__init__(self, self._triangles)

        # Initialize this class' interface with Adolphus.
        occ_triangle = []
        for triangle in self.faces:
            occ_triangle.append(OcclusionTriangle(triangle.vertices, Pose(), None))
        SceneObject.__init__(self, name, pose=pose, mount_pose=mount_pose, \
            mount=mount, primitives=[], triangles=occ_triangle)
        if VISUAL_ENABLED:
            self.visualize()

    def _import_raw(self, file):
        """\
        Import a raw triangulated mesh.
        """
        self._triangles = []
        with open(file, 'r') as f:
            for line in f.readlines():
                line = line[:-2].split(' ')
                num = []
                for item in line:
                    num.append(float(item))
                points = [Point(num[0], num[1], num[2]), \
                          Point(num[3], num[4], num[5]), \
                          Point(num[6], num[7], num[8])]
                self._triangles.append(Triangle(*points))

    def _import_dae(self, file):
        """\
        Import a collada object.
        """
        solid = Collada(file)
        self._triangles = []
        for geometry in solid.geometries:
            for triangle_set in geometry.primitives:
                for triangle in triangle_set:
                    v = triangle.vertices
                    try:
                        points = [Point(v[0][0], v[0][1], v[0][2]), \
                                  Point(v[1][0], v[1][1], v[1][2]), \
                                  Point(v[2][0], v[2][1], v[2][2])]
                        self._triangles.append(Triangle(*points))
                    except IndexError:
                        pass

    def scale(self, value):
        """\
        Scale the model by a factor on its x, y, and z coordinates.

        @param value: The scalar factor.
        @type value: C{float}
        """
        for i in range(len(self._triangles)):
            for j in [0,1,2]:
                self._triangles[i][j] *= value
        self.compute_topology()
        self._single_c = False

        # Update visualization by re-drawing the triangles.
        try:
            for triangle in self.triangles:
                triangle.visible = False
        except:
            pass
        del self.triangles
        occ_triangle = []
        for triangle in self.faces:
            occ_triangle.append(OcclusionTriangle(triangle.vertices, Pose(), None))
        try:
            self.triangles = set()
        except AttributeError:
            # Child class handles triangles separately.
            pass
        else:
            for triangle in occ_triangle:
                triangle.mount = self
                self.triangles.add(triangle)
            self._triangles_view = False
        if VISUAL_ENABLED:
            self.visualize()

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
            self._single = PointCache()
            for triangle in self.faces:
                average = avg_points(triangle.vertices)
                angles = triangle.normal_angles()
                self._single[DirectionalPoint(average.x, average.y, average.z, \
                    angles[0], angles[1])] = 1.0
            self._single_c = True
            return self._single

    def _save_raw(self, name):
        """\
        Save the model as a raw file.
        """
        f = open(name+'.raw', 'w')
        for triangle in self._triangles:
            line = ''
            for point in triangle:
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
        for vertex in self.vertices:
            vert_floats.append(vertex.x)
            vert_floats.append(vertex.y)
            vert_floats.append(vertex.z)
        for normal in self.normals:
            norm_floats.append(normal.x)
            norm_floats.append(normal.y)
            norm_floats.append(normal.z)
        indices = array(self.collada_indices)
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
