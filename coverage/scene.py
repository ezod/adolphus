"""\
Scene model module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from numpy import arange
from math import pi
from fuzz import RealRange, FuzzySet, FuzzyElement

from geometry import Point, DirectionalPoint, Pose, Plane

try:
    import visual
    VIS = True
except ImportError:
    VIS = False

class Scene(set):
    """\
    Discrete spatial-directional range with occlusion class.
    """
    def __init__(self, iterable = set()):
        """\
        Constructor.

        @param iterable: The initial set of opaque scene planes.
        @type iterable: C{iterable}
        """
        for plane in iterable:
            if not isinstance(plane, Plane):
                raise TypeError("only planes can be added")
        set.__init__(self, iterable)

    def add(self, plane):
        """\
        Add an opaque plane to the scene.

        @param plane: The item to add.
        @type plane: C{object}
        """
        if not isinstance(plane, Plane):
            raise TypeError("only planes can be added")
        set.add(self, plane)

    def occluded(self, p, cam = Point()):
        """\
        Return whether the specified point is occluded from the camera
        viewpoint (by opaque scene planes).
        """
        for plane in self:
            if plane.intersection(p, cam) is not None:
                return True
        return False

    def visualize(self, color = (1, 1, 1)):
        """\
        Visualize the opaque scene objects.

        @param color: The color of opaque scene objects.
        @type color: C{tuple}
        """
        if not VIS:
            raise ImportError("visual module not loaded")
        for plane in self:
            plane.visualize(color = color)
