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

import yaml


class VisualizationError(Exception):
    pass


class Sprite(visual.frame):
    """\
    Sprite class.
    """
    def __init__(self, definition, frame=None):
        """\
        Constructor.

        @param filename: The YAML file to load the sprite from.
        @type filename: C{str}
        @param frame: The parent frame for the sprite.
        @type frame: L{visual.frame}
        """
        super(Sprite, self).__init__(frame=frame)
        self.primitives = definition['primitives']
        self.members = []
        for primitive in self.primitives:
            ptype = primitive['type']; del primitive['type']
            primitive['material'] = getattr(visual.materials, primitive['material'])
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

    def __init__(self, definitions=[]):
        """\
        Constructor.

        @param definitions: A list of definition files for sprites.
        @type definitions: C{list} of C{string}
        """
        for definition in definitions:
            if isinstance(definition, str):
                definition = yaml.load(open(definition))
        self.definitions = definitions
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
            self.actuals[display] = [Sprite(definition) for definition in self.definitions]
        self.update_visualization()

    def update_visualization(self):
        """\
        Update this visualizable.
        """
        for display in self.actuals.keys():
            for sprite in self.actuals[display]:
                try:
                    sprite.transform(self.pose)
                except AttributeError:
                    pass
                sprite.opacity = self.opacity
