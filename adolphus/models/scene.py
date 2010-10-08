from ..geometry import Point, Pose, Plane
from ..visualization import visual, VisualizationObject, VisualizationError


class VisionPlatform(object):
    """\
    Scene object for the vision platform.
    """
    def __init__(self, pose=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        self.pose = pose
        self.center = pose.T

    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this object.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the object.
        @rtype: L{Point}
        """
        return None
        # TODO: intersection

    def visualize(self, scale=None, color=None, opacity=1.0):
        """\
        Create a 3D visual model of the object.

        @param scale: Not used.
        @type scale: C{object}
        @param color: Not used.
        @type color: C{object}
        @param opacity: The opacity of the visualization.
        @type opacity: C{float}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        self.vis = VisualizationObject(self)
        for y in [20, 900]:
            for z in [20, 1280, 2550]:
                self.vis.add('bar%d%d' % (y, z), visual.box(frame=self.vis,
                    pos=(885, y, z), width=40, height=40, length=1770,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
                if z == 1280:
                    continue
                self.vis.add('wheel%d%d' % (y, z), visual.cylinder(frame=\
                    self.vis, pos=(-60, y - 12, z), axis=(0, 24, 0), radius=50,
                    color=(0.1, 0.1, 0.1), opacity=opacity,
                    material=visual.materials.rough))
        for x in [20, 510, 1750]:
            for y in [20, 900]:
                self.vis.add('bar%d%d' % (x, y), visual.box(frame=self.vis,
                    pos=(x, y, 1285), length=40, height=40, width=2570,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
        for x in [20, 510, 1750]:
            for z in [20, 1280, 2550]:
                self.vis.add('bar%d%d' % (x, z), visual.box(frame=self.vis,
                    pos=(x, 460, z), length=40, width=40, height=920,
                    color=(1, 0.5, 0), opacity=opacity,
                    material=visual.materials.plastic))
        self.vis.add('top', visual.box(frame=self.vis, pos=(1775, 460, 1285),
            length=10, width=2530, height=860, color=(0.1, 0.1, 0.1),
            opacity=opacity, material=visual.materials.plastic))
        self.vis.add('steel', visual.box(frame=self.vis, pos=(535, 460, 1590),
            length=10, width=1900, height=860, color=(0.6, 0.6, 0.6),
            opacity=opacity, material=visual.materials.rough))
        self.vis.add('wood', visual.box(frame=self.vis, pos=(49.5, 460, 650),
            length=19, width=1240, height=860, color=(0.91, 0.85, 0.64),
            opacity=opacity, material=visual.materials.wood))
        self.vis.transform(self.pose)


class CheckerCalibrationBoard(object):
    """\
    Scene object for the checkerboard pattern calibration target.
    """
    def __init__(self, pose=None):
        """\
        Constructor.

        @param pose: The pose of the plane normal (from z-hat).
        @type pose: L{Pose}
        """
        self.pose = pose
        self.plane = Plane(pose=pose, x=(-138, 174), y=(-111, 115))
        self.center = pose.T
    
    def intersection(self, pa, pb):
        """\
        Return the 3D point of intersection (if any) of the line segment
        between the two specified points and this object.

        @param pa: The first vertex of the line segment.
        @type pa: L{Point}
        @param pb: The second vertex of the line segment.
        @type pb: L{Point}
        @return: The point of intersection with the object.
        @rtype: L{Point}
        """
        pr = self.plane.intersection(pa, pb)
        prlocal = (-self.pose).map(pr)
        if prlocal.x > 276 and prlocal.y > -55 and prlocal.y < 57:
            return None
        return pr

    def visualize(self, scale=None, color=None, opacity=1.0):
        """\
        Create a 3D visual model of the object.

        @param scale: Not used.
        @type scale: C{object}
        @param color: Not used.
        @type color: C{object}
        @param opacity: The opacity of the visualization.
        @type opacity: C{float}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        self.vis = VisualizationObject(self)
        self.vis.add('bm', visual.box(frame=self.vis, pos=(0, 0, 2),
            length=2, width=226, height=276, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        self.vis.add('bl', visual.box(frame=self.vis, pos=(0, 156, -83),
            length=2, width=56, height=36, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        self.vis.add('br', visual.box(frame=self.vis, pos=(0, 156, 86),
            length=2, width=58, height=36, color=(0.74, 0.67, 0.53),
            opacity=opacity, material=visual.materials.wood))
        for x in range(-7, 7):
            for y in range(-5, 5):
                self.vis.add('c%d%d' % (x, y), visual.box(frame=self.vis,
                    pos=(1, 9.5 + 19 * x, 9.5 + 19 * y), length=1.2, width=19,
                    height=19, color=((x + y) % 2, (x + y) % 2, (x + y) % 2),
                    opacity=opacity))
        self.vis.transform(self.pose)
