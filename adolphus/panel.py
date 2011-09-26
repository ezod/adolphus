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
    def __init__(self, parent=None, host=None, port=None):
        """\
        Constructor.
        """
        super(Panel, self).__init__()
        try:
            self.set_screen(parent.get_screen())
        except AttributeError:
            self.connect('destroy', lambda *w: gtk.main_quit())

        # basics
        self.set_title('Adolphus Panel')
        self.connect('delete_event', self._delete_event)
        self.connect('destroy', self._destroy)
        vbox = gtk.VBox()
        self.add(vbox)

        menubar = gtk.MenuBar()
        menubar.set_border_width(5)
        vbox.pack_start(menubar, expand=False)
        menuc = gtk.MenuItem('File')
        menu = gtk.Menu()
        menui = gtk.ImageMenuItem('Connect...')
        img = gtk.image_new_from_stock(gtk.STOCK_NETWORK, gtk.ICON_SIZE_MENU)
        menui.set_image(img)
        menui.connect('activate', self._connect)
        menu.add(menui)
        menui = gtk.ImageMenuItem(gtk.STOCK_QUIT)
        menui.connect('activate', self._destroy)
        menu.add(menui)
        menuc.set_submenu(menu)
        menubar.add(menuc)

        vpaned = gtk.VPaned()
        vpaned.set_border_width(5)
        vbox.pack_start(vpaned, True, True)
        frame = gtk.Frame()
        frame.set_shadow_type(gtk.SHADOW_IN)
        vpaned.add1(frame)
        frame = gtk.Frame()
        frame.set_shadow_type(gtk.SHADOW_IN)
        vpaned.add2(frame)

        # show panel
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
        self.sock.send('pickle#')

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

    def _connect(self, widget, data=None):
        dialog = gtk.Dialog('Connect to Adolphus', self, 0, (gtk.STOCK_OK, gtk.RESPONSE_OK, gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL))
        dialog.set_border_width(5)
        table = gtk.Table(2, 2)
        table.set_row_spacings(5)
        table.set_col_spacings(5)
        dialog.vbox.pack_start(table, True, True, 0)
        label = gtk.Label('_Host')
        label.set_use_underline(True)
        table.attach(label, 0, 1, 0, 1)
        host = gtk.Entry()
        host.set_text('localhost')
        table.attach(host, 1, 2, 0, 1)
        label.set_mnemonic_widget(host)
        label = gtk.Label('_Port')
        label.set_use_underline(True)
        table.attach(label, 0, 1, 1, 2)
        port = gtk.Entry()
        table.attach(port, 1, 2, 1, 2)
        label.set_mnemonic_widget(port)
        dialog.show_all()
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            self.ad_connect(host.get_text(), int(port.get_text()))
        dialog.destroy()
