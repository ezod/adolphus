"""\
Interactive GTK+ panel module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

try:
    import cPickle as pickle
except ImportError:
    import pickle

import pygtk
pygtk.require('2.0')
import gtk
import socket


class Panel(gtk.Window):
    """\
    Control panel window.
    """
    def __init__(self, host=None, port=None):
        """\
        Constructor.
        """
        super(Panel, self).__init__()

        # basics
        self.set_title('Adolphus Panel')
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

        # connect to experiment
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        if port:
            self.ad_connect(host, port)

    @staticmethod
    def main():
        """\
        Main event loop.
        """
        gtk.main()

    def ad_connect(self, host, port):
        """\
        Connect to Adolphus.
        
        @param host: The hostname (defaults to local host).
        @type host: C{str}
        @param port: THe port to connect on.
        @type port: C{int}
        """
        self.sock.connect((host or 'localhost', port))
        self.sock.send('1#')

    def ad_command(self, cmd):
        """\
        Send a command.

        @param cmd: The command to send.
        @type cmd: C{str}
        """
        self.sock.send('%s#' % cmd)
        self.sock.recv(256)

    def _delete_event(self, widget, data=None):
        return False

    def _destroy(self, widget, data=None):
        try:
            self.ad_command('exit')
        except socket.error:
            pass
        self.sock.close()
        gtk.main_quit()
