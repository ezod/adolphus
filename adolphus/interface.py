"""\
Visual interface module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import visual
import sys


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
        visual.display.__init__(self, title = title, center = center, up = (0,
            0, 1), background = (1, 1, 1), foreground = (0.3, 0.3, 0.3))


class Experiment(object):
    """\
    Experiment class.
    """
    def __init__(self, model, display = Display()):
        """\
        Constructor.

        @param model: The multi-camera model to use.
        @type model: L{coverage.MultiCamera}
        @param display: The 3D display to use.
        @type display: L{Display}
        """
        self.model = model
        self.display = display
        self.display.select()

    def run(self):
        """\
        Run this experiment.
        """
        self.model.update_model()
        self.model.visualize(scale = 30)
        cam_vis = [primitive for objects in [vis.objects for vis in \
            [self.model[camera].vis for camera in self.model]] \
            for primitive in objects]
        while True:
            visual.rate(50)
            if self.display.mouse.events:
                m = self.display.mouse.getevent()
                if m.click == "left" and m.pick in cam_vis:
                    if m.ctrl:
                        m.pick.frame.camera.visualize_fov_toggle(scale = 1500)
                    else:
                        m.pick.frame.camera.active = not m.pick.frame.camera.active
                        m.pick.frame.camera.update_visualization()
            if self.display.kb.keys:
                k = self.display.kb.getkey()
                if k == 'f2':
                    print "Updating discrete fuzzy coverage model... ",
                    sys.stdout.flush()
                    self.model.update_model()
                    self.model.update_visualization()
                    print "done."
