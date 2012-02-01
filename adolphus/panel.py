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
        Populate this tree view with the hierarchy of scene objects and tasks
        from an Adolphus model.

        @param hierarchy: Hierarchy of scene objects and tasks from Adolphus.
        @type hierarchy: C{dict}
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
        Get the icon graphic file location for a given object type.

        @param objtype: The object type.
        @type objtype: C{str}
        @return: The resource icon file location.
        @rtype: C{str}
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

        # menu bar
        menubar = gtk.MenuBar()
        menubar.set_border_width(5)
        vbox.pack_start(menubar, expand=False)
        menuc = gtk.MenuItem('File')
        menu = gtk.Menu()
        menui = gtk.MenuItem('Load Model...')
        menui.connect('activate', self._loadmodel)
        menu.add(menui)
        menu.add(gtk.SeparatorMenuItem())
        menui = gtk.ImageMenuItem(gtk.STOCK_QUIT)
        menui.connect('activate', self._destroy)
        menu.add(menui)
        menuc.set_submenu(menu)
        menubar.add(menuc)
        menuc = gtk.MenuItem('View')
        menu = gtk.Menu()
        menui = gtk.MenuItem('Clear Points')
        menui.connect('activate', self._clear)
        menu.add(menui)
        menu.add(gtk.SeparatorMenuItem())
        menui = gtk.CheckMenuItem('Camera Names')
        menui.connect('activate', self._cameranames)
        menu.add(menui)
        menui = gtk.CheckMenuItem('Axes')
        menui.connect('activate', self._axes)
        menu.add(menui)
        menui = gtk.CheckMenuItem('Center Dot')
        menui.connect('activate', self._centerdot)
        menu.add(menui)
        menu.add(gtk.SeparatorMenuItem())
        menui = gtk.MenuItem('Set Center...')
        menui.connect('activate', self._setcenter)
        menu.add(menui)
        menuc.set_submenu(menu)
        menubar.add(menuc)

        # main panel boxes
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
            except (EOFError, pickle.UnpicklingError):
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

    def _loadmodel(self, widget, data=None):
        dialog = gtk.FileChooserDialog('Load Model...', self,
            gtk.FILE_CHOOSER_ACTION_OPEN, (gtk.STOCK_CANCEL,
            gtk.RESPONSE_CANCEL, gtk.STOCK_OPEN, gtk.RESPONSE_OK))
        dialog.set_default_response(gtk.RESPONSE_OK)
        filt = gtk.FileFilter()
        filt.set_name('Adolphus Models')
        filt.add_pattern('*.yaml')
        dialog.add_filter(filt)
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            self.ad_command('loadmodel %s' % dialog.get_filename())
            self.populate_object_tree()
        dialog.destroy()

    def _select_object(self, widget, data=None):
        model, selected = widget.get_selected_rows()
        try:
            self.ad_command('select %s' % \
                model.get_value(model.get_iter(selected[0]), 1))
        except IndexError:
            self.ad_command('select')

    def _setcenter(self, widget, data=None):
        c = self.ad_command('getcenter')
        dialog = gtk.Dialog('Set Center', self, 0, (gtk.STOCK_OK,
            gtk.RESPONSE_OK, gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL))
        dialog.set_border_width(5)
        table = gtk.Table(2, 3)
        table.set_row_spacings(5)
        table.set_col_spacings(5)
        dialog.vbox.pack_start(table, True, True, 0)
        table.attach(gtk.Label('X'), 0, 1, 1, 2)
        table.attach(gtk.Label('Y'), 1, 2, 1, 2)
        table.attach(gtk.Label('Z'), 2, 3, 1, 2)
        p = []
        for i in range(3):
            p.append(gtk.Adjustment(c[i], -1e5, 1e5, 30))
            sb = gtk.SpinButton(p[-1])
            sb.set_numeric(True)
            sb.set_alignment(0.5)
            table.attach(sb, i, i + 1, 0, 1)
        dialog.show_all()
        response = dialog.run()
        if response == gtk.RESPONSE_OK:
            self.ad_command('setcenter %d %d %d' % \
                tuple([pv.get_value() for pv in p]))
        dialog.destroy()

    def _clear(self, wiget, data=None):
        self.ad_command('clear')

    def _cameranames(self, wiget, data=None):
        self.ad_command('cameranames')

    def _axes(self, wiget, data=None):
        self.ad_command('axes')

    def _centerdot(self, wiget, data=None):
        self.ad_command('centerdot')
