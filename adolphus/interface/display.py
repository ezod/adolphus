"""\
Visual display module.

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
        """
        display.__init__(self, title = title, center = center,
                         background = (1, 1, 1), foreground = (0.3, 0.3, 0.3))
        self.visible = True
