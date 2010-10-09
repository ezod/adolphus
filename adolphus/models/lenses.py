from ..visualization import  visual, VisualizationObject, VisualizationError


class ComputarM3Z1228CMP(VisualizationObject):
    """\
    Computar M3Z1228C-MP lens model.
    """
    def __init__(self, parent, frame=None):
        if not visual:
            raise VisualizationError("visual module not loaded")
        VisualizationObject.__init__(self, parent, frame)
        self.add('back', visual.cylinder(frame=self, pos=(0, 0, 9), radius=20,
            axis=(0, 0, 22), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('front', visual.cylinder(frame=self, pos=(0, 0, 31), radius=18,
            axis=(0, 0, 25), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))
        self.add('glass', visual.cylinder(frame=self, pos=(0, 0, 56), radius=17,
            axis=(0, 0, 2), color=(0.3, 0.7, 0.8), opacity=0.5,
            material=visual.materials.plastic))
        self.add('ring', visual.ring(frame=self, pos=(0, 0, 57), radius=17.5,
            axis=(0, 0, 1), color=(0.1, 0.1, 0.1),
            material=visual.materials.plastic))

    def fade(self, on):
        self.members['glass'].opacity = on and 0.1 or 0.5
        for member in set(self.members.keys()) - set(['glass']):
            self.members[member].opacity = on and 0.2 or 1.0
