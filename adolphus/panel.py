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
from sys import stdin, stdout


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
    def __init__(self, parent=None):
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

        # populate object tree
        self.populate_object_tree()

    @staticmethod
    def main():
        """\
        Main event loop.
        """
        gtk.main()

    def ad_command(self, cmd):
        """\
        Send a command.

        @param cmd: The command to send.
        @type cmd: C{str}
        @return: The result of the command.
        @rtype: C{str}
        """
        response = ''
        stdout.write('%s\n' % cmd)
        stdout.flush()
        while True:
            response += stdin.readline()
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
        self.objecttreeview.populate(self.ad_command('objecthierarchy'))

    def _delete_event(self, widget, data=None):
        return False

    def _destroy(self, widget, data=None):
        self.ad_command('exit')
        gtk.main_quit()

    def _select_object(self, widget, data=None):
        model, selected = widget.get_selected_rows()
        try:
            self.ad_command('select %s' % \
                model.get_value(model.get_iter(selected[0]), 1))
        except IndexError:
            self.ad_command('select')
