"""\
Visualization helper module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import visual
except ImportError:
    visual = None


class VisualizationError(Exception):
    pass


class Sprite(visual.frame):
    """\
    Sprite class.
    """
    def __init__(self, primitives, frame=None):
        """\
        Constructor.

        TODO
        @param frame: The parent frame for the sprite.
        @type frame: L{visual.frame}
        """
        super(Sprite, self).__init__(frame=frame)
        self.primitives = primitives
        self.members = []
        for primitive in self.primitives:
            ptype = primitive['type']; del primitive['type']
            try:
                primitive['material'] = \
                    getattr(visual.materials, primitive['material'])
            except KeyError:
                pass
            primitive['frame'] = self
            self.members.append(getattr(visual, ptype)(**primitive))
        self._opacity = 1.0

    @property
    def opacity(self):
        """\
        Sprite opacity.
        """
        return self._opacity

    @opacity.setter
    def opacity(self, value):
        for i in range(len(self.members)):
            try:
                self.members[i].opacity = self.primitives[i]['opacity'] * value
            except KeyError:
                self.members[i].opacity = value
        self._opacity = value

    def transform(self, pose):
        """\
        Execute a 3D transformation on this sprite.

        @param pose: The pose.
        @type pose: L{geometry.Pose}
        """
        self.pos = pose.T
        self.axis = (1, 0, 0)
        self.up = (1, 0, 0)
        axis, angle = pose.R.to_axis_angle()
        self.rotate(axis=axis, angle=angle)


class Visualizable(object):
    """\
    Visualizable abstract base class.
    """
    displays = {}

    def __init__(self, primitives=[]):
        """\
        Constructor.

        @param primitives: A list of sprite primitives.
        @type primitives: C{list}
        """
        self.primitives = primitives
        self.actuals = {}
        self.opacity = 1.0

    def visualize(self):
        """\
        Visualize this visualizable.
        """
        if not visual:
            raise VisualizationError('visual module not loaded')
        for display in self.displays.keys():
            if display in self.actuals.keys():
                continue
            self.displays[display].select()
            self.actuals[display] = Sprite(self.primitives)
        self.update_visualization()

    def update_visualization(self):
        """\
        Update this visualizable.
        """
        for display in self.actuals.keys():
            try:
                self.actuals[display].transform(self.pose)
            except AttributeError:
                pass
            self.actuals[display].opacity = self.opacity
