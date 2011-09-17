"""\
Visualization helper module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import ctypes

try:
    import visual
except ImportError:
    visual = None


class VisualizationError(Exception):
    pass


vsettings = {'rate': 50, 'textsize': 6}
if os.name == 'nt':
    user32 = ctypes.windll.user32
    vsettings['textsize'] = int((user32.GetSystemMetrics(0) * \
        user32.GetSystemMetrics(1)) / 100000)


class Sprite(visual.frame):
    """\
    Sprite class.
    """
    def __init__(self, primitives, parent=None, frame=None):
        """\
        Constructor.

        @param primitives: A formatted list of visual primitives.
        @type primitives: C{dict}
        @param parent: The parent object of this sprite.
        @type parent: C{object}
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

        @param primitives: A list of sprite primitive sets.
        @type primitives: C{list} of C{dict}
        """
        for primitive in primitives:
            if 'material' in primitive:
                primitive['material'] = \
                    getattr(visual.materials, primitive['material'])
            elif 'texture' in primitive:
                primitive['material'] = visual.materials.texture(data=\
                    visual.materials.loadTGA(primitive['texture']),
                    mapping=primitive['mapping'])
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
        for display in self.actuals:
            self.actuals[display].visible = value
        self._visible = value

    def visualize(self):
        """\
        Visualize this visualizable.
        """
        if not visual:
            raise VisualizationError('visual module not loaded')
        for display in self.displays:
            if display in self.actuals:
                continue
            self.displays[display].select()
            self.actuals[display] = Sprite(self.primitives, parent=self)
            self.actuals[display].visible = self._visible
        self.update_visualization()

    def update_visualization(self):
        """\
        Update this visualizable.
        """
        for display in self.actuals:
            try:
                self.actuals[display].transform(self.pose)
            except AttributeError:
                pass
            self.actuals[display].opacity = self.opacity
