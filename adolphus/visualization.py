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
    def __init__(self, primitives, parent=None, frame=None):
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
            primitive['frame'] = self
            self.members.append(getattr(visual, primitive['type'])(**primitive))
        self._opacity = 1.0
        self.parent = parent

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
        for primitive in primitives:
            try:
                primitive['material'] = \
                    getattr(visual.materials, primitive['material'])
            except KeyError:
                pass
        self.primitives = primitives
        self.actuals = {}
        self.opacity = 1.0
        self._visible = True

    def __del__(self):
        self.visible = False

    @property
    def visible(self):
        return self._visible

    @visible.setter
    def visible(self, value):
        for display in self.actuals.keys():
            self.actuals[display].visible = value
        self._visible = value

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
            self.actuals[display] = Sprite(self.primitives, parent=self)
            self.actuals[display].visible = self._visible
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
