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


def transform(entity, pos, axis, angle):
    entity.pos = pos
    # FIXME: no idea why up = (-1, 0, 0) or why (-axis) is necessary
    entity.axis = (0, 0, 1)
    entity.up = (-1, 0, 0)
    entity.rotate(axis=(-axis).tuple, angle=angle)
