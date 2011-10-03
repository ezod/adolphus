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
import gobject
import pkg_resources
import socket


class ObjectTreeView(gtk.TreeView):
    """\
    Object tree view.
    """
    def __init__(self):
        """\
        Constructor.
        """
        super(ObjectTreeView, self).__init__()
        col = gtk.TreeViewColumn('Object')
        pixbuf = gtk.CellRendererPixbuf()
        col.pack_start(pixbuf, expand=False)
        col.add_attribute(pixbuf, 'pixbuf', 0)
        cell = gtk.CellRendererText()
        col.pack_start(cell, expand=True)
        col.add_attribute(cell, 'text', 1)
        col.set_sort_column_id(1)
        self.append_column(col)
        self.set_search_column(1)
        self.set_reorderable(True)

    def populate(self, hierarchy):
        """\
        TODO
        """
        objecttree = gtk.TreeStore(gtk.gdk.Pixbuf, gobject.TYPE_STRING)
        hiter = {}
        while hierarchy:
            for obj in hierarchy.keys():
                if not hierarchy[obj][0]:
                    hiter[obj] = objecttree.append(None,
                        (self.get_icon_pixbuf(hierarchy[obj][1]), obj))
                    del hierarchy[obj]
                else:
                    try:
                        hiter[obj] = objecttree.append(hiter[hierarchy[obj][0]],
                            (self.get_icon_pixbuf(hierarchy[obj][1]), obj))
                    except KeyError:
                        continue
                    else:
                        del hierarchy[obj]
        self.set_model(objecttree)

    def get_icon_pixbuf(self, objtype):
        """\
        TODO
        """
        try:
            return gtk.gdk.pixbuf_new_from_file_at_size(\
                pkg_resources.resource_filename(__name__, 'resources/icons/' \
                + objtype.lower() + '.png'), 16, 16)
        except Exception:
            return None


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

        self.connected = False

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
        sw = gtk.ScrolledWindow()
        sw.set_shadow_type(gtk.SHADOW_IN)
        self.objecttreeview = ObjectTreeView()
        self.objecttreeview.get_selection().connect('changed',
            self._select_object)
        sw.add(self.objecttreeview)
        vpaned.add1(sw)
        sw = gtk.Frame()
        sw.set_shadow_type(gtk.SHADOW_IN)
        vpaned.add2(sw)

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
        self.sock.settimeout(0.1)
        self.connected = True
        self.populate_object_tree()

    def ad_command(self, cmd):
        """\
        Send a command.

        @param cmd: The command to send.
        @type cmd: C{str}
        @return: The result of the command.
        @rtype: C{str}
        """
        response = ''
        self.sock.send('%s#' % cmd)
        while True:
            try:
                response += self.sock.recv(256)
            except socket.error:
                pass
            try:
                robj = pickle.loads(response)
                break
            except Exception:
                continue
        if isinstance(robj, Exception):
            raise robj
        else:
            return robj

    def populate_object_tree(self):
        if not self.connected:
            return
        self.objecttreeview.populate(self.ad_command('objecthierarchy'))

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
        dialog = gtk.Dialog('Connect to Adolphus', self, 0, (gtk.STOCK_OK,
            gtk.RESPONSE_OK, gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL))
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

    def _select_object(self, widget, data=None):
        model, selected = widget.get_selected_rows()
        try:
            self.ad_command('select %s' % \
                model.get_value(model.get_iter(selected[0]), 1))
        except IndexError:
            self.ad_command('select')
