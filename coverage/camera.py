"""\
Coverage model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import sqrt, sin, cos, atan, pi
from numbers import Number
from itertools import combinations
from fuzz import IndexedSet, TrapezoidalFuzzyNumber, FuzzySet, FuzzyElement

from coverage.geometry import Point, DirectionalPoint, Pose

try:
    import visual
    VIS = True
except ImportError:
    VIS = False


class Camera( object ):
    """\
    Single-camera model, using continous fuzzy sets.
    """
    def __init__(self, name, A, f, s, o, dim, zS, gamma, r1, r2, cmax, zeta,
                 pose = Pose()):
        """\
        Constructor.

        @param name: A name for the camera.
        @type name: C{str}
        @param A: Aperture size (intrinsic).
        @type A: C{float}
        @param f: Focal length (intrinsic).
        @type f: C{float}
        @param s: Effective pixel size (intrinsic).
        @type s: C{tuple} of C{float}
        @param o: Pixel coordinates of principal point (intrinsic).
        @type o: C{tuple} of C{float}
        @param dim: Sensor pixel dimensions (intrinsic).
        @type dim: C{int}
        @param zS: Subject distance (intrinsic).
        @type zS: C{float}
        @param gamma: Fuzzification value for visibility (application).
        @type gamma: C{float}
        @param r1: Fully acceptable resolution (application).
        @type r1: C{float}
        @param r2: Acceptable resolution (application).
        @type r2: C{float}
        @param cmax: Maximum acceptable circle of confusion (application).
        @type cmax: C{float}
        @param zeta: Fuzzification value for direction (application).
        @type zeta: C{float}
        @param pose: Pose of the camera in space (optional).
        @type pose: L{geometry.Pose}
        """
        self.name = name

        if isinstance(s, Number):
            s = (s, s)

        # fuzzy sets for visibility
        self.Cv, al, ar = [], [], []
        for i in range(2):
            al.append(2.0 * atan((o[i] * s[i]) / (2.0 * f)))
            ar.append(2.0 * atan(((dim[i] - o[i]) * s[i]) / (2.0 * f)))
            g = (gamma / float(dim[i])) * 2.0 * sin((al[i] + ar[i]) / 2.0)
            self.Cv.append(TrapezoidalFuzzyNumber((-sin(al[i]) + g,
                sin(ar[i]) - g), (-sin(al[i]), sin(ar[i]))))

        # fuzzy set for resolution
        mr = min([float(dim[i]) / (2.0 * sin((al[i] + ar[i]) / 2.0)) \
                  for i in range(2)])
        zr1 = (1.0 / r1) * mr
        zr2 = (1.0 / r2) * mr
        self.Cr = TrapezoidalFuzzyNumber((0, zr1), (0, zr2))

        # fuzzy set for focus
        zl = (A * f * zS) / (A * f + min(s) * (zS - f))
        zr = (A * f * zS) / (A * f - min(s) * (zS - f))
        zn = (A * f * zS) / (A * f + cmax * (zS - f))
        zf = (A * f * zS) / (A * f - cmax * (zS - f))
        self.Cf = TrapezoidalFuzzyNumber((zl, zr), (zn, zf))

        # fuzzifier for direction
        self.zeta = zeta
        
        # pose
        self.pose = pose

    def __hash__(self):
        """\
        Hash function (based on name).
        """
        return hash(self.name)

    def __eq__(self, other):
        """\
        Equality function (based on name).

        @param other: The other operand.
        @type other: L{Camera}
        @return: True if equal, false otherwise.
        @rtype: C{bool}
        """
        if not isinstance(other, Camera):
            return False
        return self.name == other.name

    def mu(self, dpoint):
        """\
        Return the membership degree (coverage) for a directional point.
    
        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        if not isinstance(dpoint, Point):
            raise TypeError("invalid point")

        campoint = (-self.pose).map(dpoint)

        # visibility
        try:
            mu_v = min([Cv.mu(campoint.x / campoint.z) for Cv in self.Cv])
        except ZeroDivisionError:
            mu_v = 0.0

        # resolution
        mu_r = self.Cr.mu(campoint.z)

        # focus
        mu_f = self.Cf.mu(campoint.z)

        # direction
        if isinstance(campoint, DirectionalPoint):
            r = sqrt(campoint.x ** 2 + campoint.y ** 2)
            try:
                terma = (campoint.y / r) * sin(campoint.eta) + \
                        (campoint.x / r) * cos(campoint.eta)
            except ZeroDivisionError:
                terma = 1.0
            try:
                termb = atan(r / campoint.z)
            except ZeroDivisionError:
                termb = pi / 2.0
            mu_d = min(max((float(campoint.rho) - ((pi / 2.0) + terma \
                * termb)) / self.zeta, 0.0), 1.0)
        else:
            mu_d = 1.0

        # algebraic product intersection
        return mu_v * mu_r * mu_f * mu_d

    def visualize(self, scale = 1.0, color = (1, 1, 1)):
        """\
        Plot the camera in a 3D visual model.

        @param scale: The scale of the camera.
        @type scale: C{float}
        @param color: The color in which to plot the point.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError("visual module not loaded")
        visual.pyramid(pos = self.pose.T.tuple, axis = \
            self.pose.map_rotate(Point(0, 0, -scale)).tuple, \
            size = (scale, scale, scale), color = color)
        # project a cylinder for debugging/validation
        # visual.cylinder( pos = self.pose.T.tuple, axis = \
        #     self.pose.map_rotate( Point( 0, 0, scale ) ).tuple, \
        #     length = 2000, radius = scale / 10.0 )


class MultiCamera(IndexedSet):
    """\
    Abstract base class for multi-camera model.
    """
    def __init__(self, scene, cameras = set(), directional = True, dense = True):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        @param dense: Evaluate at every discrete point in the scene.
        @type dense: C{bool}
        """
        if self.__class__ is MultiCamera:
            raise NotImplementedError("please use one of the subclasses")
        self.directional = directional
        self.dense = dense
        self.model = FuzzySet()
        self.scene = scene
        self.inscene = {}
        IndexedSet.__init__(self, 'name', cameras)
        for camera in self:
            self._update_inscene(camera.name)

    def add(self, item):
        """\
        Add a camera to the set. Overrides the base class by verifying that
        the item to add is a Camera object, and updating the in-scene set.

        @param item: The item (camera) to add.
        @type item: L{Camera}
        """
        if isinstance(item, Camera):
            IndexedSet.add(self, item)
            self._update_inscene(item.name)

    def update_point(self, dpoint):
        """\
        Update a single (directional) point in the model, for all cameras, with
        occlusion.

        @param dpoint: The directional point.
        @type dpoint: L{DirectionalPoint}
        """
        for key in self.keys():
            mu = self[key].mu(dpoint)
            if not self.scene.occluded(dpoint, self[key].pose.T):
                self.inscene[key] |= FuzzySet([FuzzyElement(dpoint, mu)])

    def _update_inscene(self, key):
        """\
        Update an individual in-scene camera fuzzy set (with occlusion).

        @param key: The name of the camera to update.
        @type key: C{str}
        """
        if not self.dense:
            if not self.inscene.has_key(key):
                self.inscene[key] = FuzzySet()
            return
        dpoints = []
        for dpoint in self.scene.generate_points(d = self.directional \
                                                 and 'all' or None):
            mu = self[key].mu(dpoint)
            if mu > 0 and not self.scene.occluded(dpoint, self[key].pose.T):
                dpoints.append(FuzzyElement(dpoint, mu))
        self.inscene[key] = FuzzySet(dpoints)

    def performance(self):
        """\
        Return the coverage performance of this multi-camera network.
        """
        return self.model.overlap(self.scene.D)

    def visualize(self, scale = 1.0, color = (1, 1, 1), points = True):
        """\
        Visualize all cameras and the directional points of the coverage model
        (with opacity reflecting degree of coverage).

        @param scale: The scale of the individual elements.
        @type scale: C{float}
        @param color: The color of cameras.
        @type color: C{tuple}
        """
        if not VIS:
            raise NotImplementedError("visual module not loaded")
        for camera in self:
            camera.visualize(scale = scale, color = color)
        self.scene.visualize(color = color)
        if points:
            for dpoint in self.model.keys():
                dpoint.visualize(scale = scale, color = (1, 0, 0),
                                 opacity = self.model[dpoint].mu)


class MultiCameraSimple(MultiCamera):
    """\
    Simple (single-camera coverage) multi-camera model.
    """
    def __init__(self, scene, cameras = set(), directional = True, dense = True):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        @param dense: Evaluate at every discrete point in the scene.
        @type dense: C{bool}
        """
        MultiCamera.__init__(self, scene, cameras, directional, dense)

    def update_model(self):
        """\
        Update the simple multi-camera network discrete spatial-directional
        fuzzy set (coverage model).
        """
        self.model = FuzzySet()
        for camera in self:
            self.model |= self.inscene[camera.name]

    def mu(self, dpoint):
        """\
        Return the individual membership degree of a point using the continuous
        in-scene camera coverage models (so the point does not necessarily need
        to fall on the discrete grid).

        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        views = []
        for camera in self:
            if not self.scene.occluded(dpoint, camera.pose.T):
                views.append(camera)
        try:
            return max([camera.mu(dpoint) for camera in views])
        except ValueError:
            return 0.0


class MultiCamera3D(MultiCamera):
    """\
    3D (dual-camera coverage) multi-camera model.
    """
    def __init__(self, scene, cameras = set(), directional = True, dense = True):
        """\
        Constructor.

        @param scene: The discrete scene model.
        @type scene: L{Scene}
        @param cameras: The initial set of cameras.
        @type cameras: C{set}
        @param directional: Use directional points if true.
        @type directional: C{bool}
        @param dense: Evaluate at every discrete point in the scene.
        @type dense: C{bool}
        """
        MultiCamera.__init__(self, scene, cameras, directional, dense)

    def update_model(self):
        """\
        Update the 3D multi-camera network discrete spatial-directional fuzzy
        set (coverage model).
        """
        self.model = FuzzySet()
        pairs = [self.inscene[pair[0]] & \
                 self.inscene[pair[1]] \
                 for pair in combinations(self.keys(), 2)]
        for pair in pairs:
            self.model |= pair

    def mu(self, dpoint):
        """\
        Return the individual membership degree of a point using the continuous
        in-scene camera coverage models (so the point does not necessarily need
        to fall on the discrete grid).

        @param dpoint: The (directional) point to test.
        @type dpoint: L{geometry.Point}
        @return: The membership degree (coverage) of the point.
        @rtype: C{float}
        """
        # TODO: account for occlusion
        return max([min(self[pair[0]].mu(dpoint), \
                        self[pair[1]].mu(dpoint)) \
                    for pair in combinations(self.keys(), 2)])
