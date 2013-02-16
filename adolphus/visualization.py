"""\
Visualization helper module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os

VISUAL_ENABLED = True
try:
    from .sprite import Sprite
except ImportError:
    VISUAL_ENABLED = False


class VisualizationError(Exception):
    "Visualization failed."
    pass


if os.name == 'posix':
    # TODO: Test on Linux.
    # Apparently this only works on OSX > 10.8. For it to work on OSX 10.8
    # the X11 system is requred, which needs to be installed :(
    screen = os.popen("xdpyinfo  | grep 'dimensions:'").readlines()[0]
    screen = screen.split( )[1].split('x')
    width = float(screen[0])
    height = float(screen[1])
elif os.name == 'nt':
    import ctypes
    user32 = ctypes.windll.user32
    width = float(user32.GetSystemMetrics(0))
    height = float(user32.GetSystemMetrics(1))
else:
    width = 1280.0
    height = 720.0

# This relationship determines the appropiate text size for the
# corresponsing screen aspect ratio. The relationship was calibrated
# by testing several screen resolutions and text sizes.
size = int(4.5**(width/height)
VISUAL_SETTINGS = {'rate': 50, 'textsize': size, 'scale': 1.0}


class Visualizable(object):
    """\
    Visualizable abstract base class.

    A L{Visualizable} object is a higher-level visual representation manager
    which instantiates one or more L{Sprite} objects (one per Visual display).
    The class itself maintains a dictionary of active displays, while each
    instance maintains its own set of "actuals" (visualized sprites).
    """
    displays = {}

    def __init__(self, primitives=list()):
        """\
        Constructor.

        @param primitives: A formatted list of visual primitives.
        @type primitives: C{list} of C{dict}
        """
        self.primitives = primitives
        self.opacity = 1.0
        self._visible = True
        try:
            self.actuals = {}
        except AttributeError:
            pass

    def __del__(self):
        self.visible = False

    def get_visible(self):
        """\
        Visibility of this visualizable.
        """
        return self._visible

    def set_visible(self, value):
        for display in self.actuals:
            self.actuals[display].visible = value
        self._visible = value

    visible = property(get_visible, set_visible)

    def highlight(self, color=(0, 1, 0)):
        """\
        Highlight this visualizable with a bright uniform color.

        @param color: The color of the highlight.
        @type color: C{tuple}
        """
        for display in self.actuals:
            self.actuals[display].highlight(color)

    def unhighlight(self):
        """\
        Unhighlight this visualizable (if highlighted).
        """
        for display in self.actuals:
            self.actuals[display].unhighlight()

    def visualize(self):
        """\
        Visualize this visualizable. Creates sprites in any "new" displays,
        and updates the visualization in all displays.
        """
        if not VISUAL_ENABLED:
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
        Update this visualizable in all displays. If it has a pose (i.e.
        subclasses or ducktypes L{Posable}), transforms its sprites to the
        current pose.
        """
        for display in self.actuals:
            try:
                self.actuals[display].transform(self.pose)
            except AttributeError:
                pass
            self.actuals[display].opacity = self.opacity
