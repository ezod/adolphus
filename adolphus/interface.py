"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import visual


class Display(visual.display):
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
        visual.display.__init__(self, title = title, center = center, up = (0, 0, 1),
                         background = (1, 1, 1), foreground = (0.3, 0.3, 0.3))
        self.select()


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, display = Display()):
        """\
        Constructor.

        @param display: The 3D display to use.
        @type display: L{Display}
        """
        pass

    def run(self):
        """\
        Run this experiment.
        """
        self.display.visible = True
        while True:
            visual.rate(50)
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.click == "left" and m.pick in self.cameras:
                    m.pick.visualize_fov_toggle()
