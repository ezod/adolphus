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
    Visualization object class.
    """
    def __init__(self, parent, frame=None,
                 properties={'scale': 1.0, 'color': (1, 1, 1), 'opacity': 1.0}):
        """\
        Constructor.

        @param parent: Reference to the parent object being visualized.
        @type parent: C{object}
        @param frame: Parent frame for this frame.
        @type frame: C{visual.frame}
        @param properties: Visualization property storage.
        @type properties: C{dict}
        """
        self.parent = parent
        self.properties = properties
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
        entity.frame = self
        self.members[name] = entity

    def transform(self, pose):
        """\
        Execute a 3D transformation on this visualization object.

        @param pose: The pose.
        @type pose: L{geometry.Pose}
        """
        self.pos = pose.T
        self.axis = (1, 0, 0)
        self.up = (1, 0, 0)
        axis, angle = pose.R.to_axis_angle()
        self.rotate(axis=axis, angle=angle)

    def fade(self, on):
        """\
        Reduce the opacity of the visualization object.

        @param on: Turn fade on or off.
        @type on: C{bool}
        """
        for member in self.members.keys():
            if isinstance(self.members[member], visual.label):
                continue
            elif isinstance(self.members[member], VisualizationObject):
                self.members[member].fade(on)
            else:
                self.members[member].opacity = on and 0.2 or 1.0
