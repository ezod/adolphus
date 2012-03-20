"""\
Sprite module. Imported conditionally by the visualization module if import of
Visual succeeds.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import visual


class Sprite(visual.frame):
    """\
    Sprite class.

    A L{Sprite} object is a frame bound to a single Visual display containing
    one or more primitives. Normally, this is used by the L{Visualizable} class
    to handle manifestations across multiple displays, but it can also be
    instantiated directly to produce visualization elements in one particular
    display.
    """
    def __init__(self, primitives, parent=None, frame=None):
        """\
        Constructor.

        @param primitives: A formatted list of visual primitives. Each element
                           of the list is a dictionary; C{type} keys the Visual
                           object class, and the remainder of the keys are
                           passed as keyword arguments to its constructor.
                           Materials and textures are dereferenced from strings
                           if necessary.
        @type primitives: C{list} of C{dict}
        @param parent: The parent object of this sprite.
        @type parent: C{object}
        @param frame: The parent frame for the sprite.
        @type frame: C{visual.frame}
        """
        super(Sprite, self).__init__(frame=frame)
        self.primitives = primitives
        self.members = []
        for primitive in self.primitives:
            if 'material' in primitive \
            and isinstance(primitive['material'], str):
                primitive['material'] = \
                    getattr(visual.materials, primitive['material'])
            elif 'texture' in primitive \
            and isinstance(primitive['texture'], str):
                primitive['material'] = visual.materials.texture(data=\
                    visual.materials.loadTGA(primitive['texture']),
                    mapping=primitive['mapping'])
            primitive['frame'] = self
            self.members.append(getattr(visual, primitive['type'])(**primitive))
        self._opacity = 1.0
        self._highlighted = False
        self.parent = parent

    def get_opacity(self):
        """\
        Sprite opacity.
        """
        return self._opacity

    def set_opacity(self, value):
        """\
        Set the opacity of this sprite.

        @param value: The value to which to set the opacity in [0, 1].
        @type value: C{float}
        """
        for i, member in enumerate(self.members):
            try:
                try:
                    member.opacity = self.primitives[i]['opacity'] * value
                except KeyError:
                    member.opacity = value
            except RuntimeError:
                pass
        self._opacity = value

    opacity = property(get_opacity, set_opacity)

    def highlight(self, color=(0, 1, 0)):
        """\
        Highlight this sprite with a bright uniform color.

        @param color: The color of the highlight.
        @type color: C{tuple}
        """
        for i, member in enumerate(self.members):
            if not self._highlighted:
                if isinstance(member.color, tuple):
                    self.primitives[i]['color'] = tuple(member.color)
                else:
                    self.primitives[i]['color'] = tuple(member.color[0])
                self.primitives[i]['material'] = member.material
                member.material = visual.materials.emissive
            member.color = color
        self._highlighted = True

    def unhighlight(self):
        """\
        Unhighlight this sprite (if highlighted).
        """
        if not self._highlighted:
            return
        for i, member in enumerate(self.members):
            member.color = self.primitives[i]['color']
            member.material = self.primitives[i]['material']
        self._highlighted = False

    def transform(self, pose):
        """\
        Execute a 3D transformation on this sprite. This sets the absolute pose
        of the sprite to the specified pose.

        @param pose: The pose.
        @type pose: L{Pose}
        """
        self.pos = tuple(pose.T)
        self.axis = (1, 0, 0)
        self.up = (1, 0, 0)
        angle, axis = pose.R.to_axis_angle()
        self.rotate(axis=tuple(axis), angle=angle)
