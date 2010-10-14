from ..geometry import Point, Pose, Plane, Posable
from ..visualization import visual, VisualizationObject, VisualizationError


class VisionPlatform(Posable):
    """\
    Scene object for the vision platform.
    """
    def __init__(self, pose=Pose(), mount=None, config=None):
        """\
        Constructor.

        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        @param config: The configuration of the object (unused).
        @type config: C{object}
        """
        Posable.__init__(self, pose, mount)

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
        planes = set()
        # top plate
        planes.add(Plane(pose=(self.pose + Pose(T=Point(1285, 460, 1775))),
            x=(-1265, 1265), y=(-430, 430)))
        # steel plate
        planes.add(Plane(pose=(self.pose + Pose(T=Point(1590, 460, 535))),
            x=(-950, 950), y=(-430, 430)))
        return reduce(lambda a, b: a or b, [plane.intersection(pa, pb) \
            for plane in planes])

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the object.

        @param scale: The scale of the visualization (optional).
        @type scale: C{float}
        @param color: The color of the visualization (optional).
        @type color: C{tuple}
        @param opacity: The opacity of the visualization.
        @type opacity: C{float}
        """
        if Posable.visualize(self, scale=scale, color=color, opacity=opacity):
            for x in [20, 1280, 2550]:
                for y in [20, 900]:
                    self.vis.add('bar%d%d' % (x, y), visual.box(frame=self.vis,
                        pos=(x, y, 885), length=40, width=1770, height=40,
                        color=(1, 0.5, 0), opacity=opacity,
                        material=visual.materials.plastic))
                    if x == 1280:
                        continue
                    self.vis.add('wheel%d%d' % (x, y), visual.cylinder(frame=\
                        self.vis, pos=(x, y - 12, -60), axis=(0, 24, 0),
                        radius=50, color=(0.1, 0.1, 0.1), opacity=opacity,
                        material=visual.materials.rough))
            for y in [20, 900]:
                for z in [20, 510, 1750]:
                    self.vis.add('bar%d%d' % (y, z), visual.box(frame=self.vis,
                        pos=(1285, y, z), length=2570, width=40, height=40,
                        color=(1, 0.5, 0), opacity=opacity,
                        material=visual.materials.plastic))
            for x in [20, 1280, 2550]:
                for z in [20, 510, 1750]:
                    self.vis.add('bar%d%d' % (x, z), visual.box(frame=self.vis,
                        pos=(x, 460, z), length=40, width=40, height=920,
                        color=(1, 0.5, 0), opacity=opacity,
                        material=visual.materials.plastic))
            self.vis.add('tp', visual.box(frame=self.vis, pos=(1285, 460, 1775),
                length=2530, width=10, height=860, color=(0.1, 0.1, 0.1),
                opacity=opacity, material=visual.materials.plastic))
            self.vis.add('sp', visual.box(frame=self.vis, pos=(1590, 460, 535),
                length=1900, width=10, height=860, color=(0.6, 0.6, 0.6),
                opacity=opacity, material=visual.materials.rough))
            self.vis.add('wp', visual.box(frame=self.vis, pos=(650, 460, 49.5),
                length=1240, width=19, height=860, color=(0.91, 0.85, 0.64),
                opacity=opacity, material=visual.materials.wood))
            self.update_visualization()
            return True
        else:
            return False


class CheckerCalibrationBoard(Posable):
    """\
    Scene object for the checkerboard pattern calibration target.
    """
    def __init__(self, pose=Pose(), mount=None, config=None):
        """\
        Constructor.

        @param pose: The pose of the object (optional).
        @type pose: L{Pose}
        @param mount: The mount of the object (optional).
        @type mount: L{Posable}
        @param config: The configuration of the object (unused).
        @type config: C{object}
        """
        Posable.__init__(self, pose, mount)
    
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
        plane = Plane(pose=pose, x=(-138, 174), y=(-111, 115))
        pr = plane.intersection(pa, pb)
        prlocal = (-self.pose).map(pr)
        if prlocal.x > 276 and prlocal.y > -55 and prlocal.y < 57:
            return None
        return pr

    def visualize(self, scale=1.0, color=(1, 1, 1), opacity=1.0):
        """\
        Visualize the object.

        @param scale: The scale of the visualization (optional).
        @type scale: C{float}
        @param color: The color of the visualization (optional).
        @type color: C{tuple}
        @param opacity: The opacity of the visualization.
        @type opacity: C{float}
        """
        if Posable.visualize(self, scale=scale, color=color, opacity=opacity):
            self.vis.add('bm', visual.box(frame=self.vis, pos=(0, 2, 0),
                length=276, width=2, height=226, color=(0.74, 0.67, 0.53),
                opacity=opacity, material=visual.materials.wood))
            self.vis.add('bl', visual.box(frame=self.vis, pos=(156, -83, 0),
                length=36, width=2, height=56, color=(0.74, 0.67, 0.53),
                opacity=opacity, material=visual.materials.wood))
            self.vis.add('br', visual.box(frame=self.vis, pos=(156, 86, 0),
                length=36, width=2, height=58, color=(0.74, 0.67, 0.53),
                opacity=opacity, material=visual.materials.wood))
            for x in range(-7, 7):
                for y in range(-5, 5):
                    self.vis.add('c%d%d' % (x, y), visual.box(frame=self.vis,
                        pos=(9.5 + 19 * x, 9.5 + 19 * y, 1),
                        color=((x + y) % 2, (x + y) % 2, (x + y) % 2),
                        length=19, width=1.2, height=19, opacity=opacity))
            self.update_visualization()
            return True
        else:
            return False
