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


class VisualizationObject(visual.frame):
    """\
    """
    def __init__(self, parent, frame=None):
        """\
        Constructor.

        @param parent: Reference to the parent object being visualized.
        @type parent: C{object}
        """
        self.parent = parent
        self.members = {}
        visual.frame.__init__(self, frame=frame)

    @property
    def primitives(self):
        primitives = []
        for object in self.objects:
            if isinstance(object, VisualizationObject):
                primitives += object.primitives
            else:
                primitives.append(object)
        return primitives

    def add(self, name, entity):
        """\
        Add an entity to the visualization object.

        @param name: The name of the entity.
        @type name: C{str}
        @param entity: The entity itself.
        @type entity: C{object}
        """
        if entity.frame is not self:
            raise ValueError("must specify this VisualizationObject as frame")
        self.members[name] = entity

    def transform(self, pos, axis, angle):
        """\
        Execute a 3D transformation on this visualization object.

        @param pos: The position.
        @type pos: C{tuple} of C{float}
        @param axis: The axis of rotation.
        @type axis: C{tuple} of C{float}
        @param angle: The angle of rotation.
        @type angle: C{float}
        """
        self.pos = pos
        self.axis = (0, 0, 1)
        self.up = (1, 0, 0)
        self.rotate(axis=(axis).tuple, angle=angle)

    def fade(self, on):
        """\
        Reduce the opacity of the visualization object.

        @param on: Turn fade on or off.
        @type on: C{bool}
        """
        for member in self.members.keys():
            if isinstance(self.members[member], VisualizationObject):
                self.members[member].fade(on)
            else:
                self.members[member].opacity = on and 0.2 or 1.0
