from ..visualization import  visual, VisualizationObject, VisualizationError


class ProsilicaEC1350(VisualizationObject):
    """\
    Prosilica EC-1350 CCD camera model.
    """
    def __init__(self, parent):
        """\
        Constructor.

        @param parent: Reference to the parent object.
        @type parent: C{object}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        VisualizationObject.__init__(self, parent)
        self.add('body', visual.box(length=45, height=33, width=35,
            pos=(0, 0, -17.5), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('ring', visual.cylinder(pos=(0, 0, 0), radius=15,
            axis=(0, 0, 4), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('mount', visual.cylinder(pos=(0, 0, 4),
            radius=12.5, axis=(0, 0, 5), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('back', visual.box(length=45, height=33, width=1,
            pos=(0, 0, -35.5), color=(0.29, 0.49, 0.69),
            material=visual.materials.rough))
        self.add('fwport', visual.box(length=6, height=12, width=10,
            pos=(-16.5, 0, -41.5), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('clport', visual.cylinder(pos=(14.5, 0, -36.5),
            radius=4.5, axis=(0, 0, -6), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))


class SICKIVPRangerED(VisualizationObject):
    """\
    SICK-IVP Ranger E/D camera model.
    """
    def __init__(self, parent):
        """\
        Constructor.

        @param parent: Reference to the parent object.
        @type parent: C{object}
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        VisualizationObject.__init__(self, parent)
        self.add('body', visual.box(length=49, height=37, width=106,
            pos=(0, 6.5, -44), color=(1, 1, 1),
            material=visual.materials.plastic))
        self.add('top', visual.box(length=23, height=13, width=106,
            pos=(0, -18.5, -44), color=(1, 1, 1),
            material=visual.materials.plastic))
        self.add('bevell', visual.cylinder(pos=(-11.5, -12, 9),
            radius=13, axis=(0, 0, -106), color=(1, 1, 1),
            material=visual.materials.plastic))
        self.add('bevelr', visual.cylinder(pos=(11.5, -12, 9),
            radius=13, axis=(0, 0, -106), color=(1, 1, 1),
            material=visual.materials.plastic))
        self.add('ecport', visual.cylinder(pos=(13.5, -9.5, -97),
            radius=6, axis=(0, 0, -15), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('pwport', visual.cylinder(pos=(13.5, 9, -97),
            radius=5, axis=(0, 0, -12), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('enport', visual.box(length=18, height=22, width=2,
            pos=(-7, 3, -98), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
