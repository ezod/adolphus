"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

from visual import display


class Display(display):
    """\
    Visual display class.
    """
    def __init__(self, title = 'Adolphus', center = (0, 0, 0)):
        """\
        Contstructor.

        @param title: Title of the window.
        @type title: C{str}
        @param center: Location of the center point.
        @type center: C{tuple} of C{float}
        """
        display.__init__(self, title = title, center = center,
                         background = (1, 1, 1), foreground = (0.3, 0.3, 0.3))
        self.visible = True


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, cameras = []):
        """\
        Constructor.
        """
        self.cameras = cameras
