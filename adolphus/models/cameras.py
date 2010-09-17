from ..visualization import  visual, VisualizationObject, VisualizationError


class ProsilicaEC1350(VisualizationObject):
    """\
    Prosilica EC-1350 CCD camera model.
    """
    def __init__(self, parent, frame=None):
        """\
        Constructor.
        """
        if not visual:
            raise VisualizationError("visual module not loaded")
        VisualizationObject.__init__(self, parent, frame)
        self.add('body', visual.box(frame=self, length=35, height=45, width=33,
            pos=(-17.5, 0, 0), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('ring', visual.cylinder(frame=self, pos=(0, 0, 0), radius=15,
            axis=(4, 0, 0), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('mount', visual.cylinder(frame=self, pos=(4, 0, 0),
            radius=12.5, axis=(5, 0, 0), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('back', visual.box(frame=self, length=1, height=45, width=33,
            pos=(-35.5, 0, 0), color=(0.29, 0.49, 0.69),
            material=visual.materials.rough))
        self.add('fwport', visual.box(frame=self, length=10, height=6, width=12,
            pos=(-41.5, 16.5, 0), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
        self.add('clport', visual.cylinder(frame=self, pos=(-36.5, -14.5, 0),
            radius=4.5, axis=(-6, 0, 0), color=(0.8, 0.8, 0.8),
            material=visual.materials.plastic))
