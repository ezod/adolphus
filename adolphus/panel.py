"""\
Interactive GTK+ panel module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import pygtk
pygtk.require('2.0')
import gtk

from .interface import Experiment


class Panel(gtk.Window):
    """\
    Control panel window.
    """
    def __init__(self, experiment=None):
        """\
        Constructor.

        @param experiment: 
        @type experiment: L{Experiment}
        """
        super(Panel, self).__init__()

        # basics
        self.set_title('Adolphus')
        self.set_border_width(10)
        self.connect('delete_event', self._delete_event)
        self.connect('destroy', self._destroy)
        vbox = gtk.VBox(False, 10)

        # interface
        close = gtk.Button('Close')
        close.connect('clicked', self._destroy)
        vbox.pack_start(close)

        # show panel
        self.add(vbox)
        self.show_all()

        # start visualization
        self.experiment = experiment or Experiment()
        self.experiment.start()

    @staticmethod
    def main():
        """\
        Main event loop.
        """
        gtk.threads_init()

    def _delete_event(self, widget, data=None):
        return False

    def _destroy(self, widget, data=None):
        gtk.main_quit()
