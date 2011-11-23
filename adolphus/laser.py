"""\
Laser range imaging module. Contains extensions and objects necessary for
modeling laser line based range imaging cameras.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from math import pi, tan

import cython
from .geometry import Angle, Pose, Point, DirectionalPoint
from .coverage import PointCache, RelevanceModel, Camera
from .posable import SceneObject


class LineLaser(SceneObject):
    """\
    Line laser class.
    """
    def __init__(self, name, fan, depth, pose=Pose(), mount=None,
                 primitives=[]):
        """\
        Constructor.

        @param name: The name of the laser.
        @type name: C{str}
        @param fan: Fan angle of the laser line.
        @type params: L{Angle}
        @param depth: Projection depth of the laser line.
        @type depth: C{float}
        @param pose: Pose of the laser in space (optional).
        @type pose: L{Pose}
        @param mount: Mount object for the laser (optional).
        @type mount: C{object}
        @param primitives: Sprite primitives for the laser.
        @type primitives: C{dict}
        """
        super(LineLaser, self).__init__(name, pose=pose, mount=mount,
            primitives=primitives)
        self._fan = Angle(fan)
        self._depth = depth
        self._generate_laservis()
        self.click_actions = {'ctrl':   'laser %s' % name,
                              'shift':  'modify %s' % name}

    @property
    def fan(self):
        """\
        Fan angle.
        """
        return self._fan

    @property
    def depth(self):
        """\
        Projection depth.
        """
        return self._depth

    def _generate_laservis(self):
        width = self.depth * tan(self.fan / 2.0)
        self.laservis = [{'type': 'curve', 'color': (1, 0, 0),
            'pos': [(0, 0, 0), (-width, 0, self.depth),
                    (width, 0, self.depth), (0, 0, 0)]}]

    def project(self, target, pitch):
        """\
        Generate a range imaging relevance model by projecting the laser line
        onto the target object.

        @param target: The target object.
        @type target: L{SceneObject}
        @param pitch: The horizontal pitch of relevance model points.
        @type pitch: C{float}
        @return: The generated range imaging relevance model.
        @rtype: L{RelevanceModel}
        """
        points = PointCache()
        width = self.depth * tan(self.fan / 2.0)
        x = int(-width / pitch) * pitch
        while x < width:
            cp = None
            origin = self.pose.map(Point((x, 0, 0)))
            end = self.pose.map(Point((x, 0, self.depth)))
            for triangle in target.triangles:
                ip = triangle.intersection(origin, end)
                if ip:
                    ip = (-self.pose).map(ip)
                else:
                    continue
                if abs(ip.x) > ip.z * tan(self.fan / 2.0):
                    continue
                elif not cp or ip.z < cp.z:
                    cp = Point(ip)
            if cp:
                points[DirectionalPoint(tuple(cp) + (pi, 0))] = 1.0
            x += pitch
        return RelevanceModel(points, mount=self)
