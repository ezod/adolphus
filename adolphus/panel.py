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
from math import pi
from sys import stdin, stdout
from copy import copy

from .coverage import Task
from .laser import RangeTask


class NumericEntry(gtk.Entry):
    """\
    Numeric text entry widget.
    """
    def __init__(self):
        super(NumericEntry, self).__init__()
        self.connect('changed', self.on_changed)
        self.set_size_request(0, -1)

    def on_changed(self, *args):
        text = self.get_text().strip()
        self.set_text(''.join([i for i in text if i in '-.e0123456789']))


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
        @type objtype: C{type}
        @return: The resource icon file location.
        @rtype: C{str}
        """
        try:
            return gtk.gdk.pixbuf_new_from_file_at_size(\
                pkg_resources.resource_filename(__name__, 'resources/icons/' \
                + objtype.__name__.lower() + '.png'), 16, 16)
        except gobject.GError:
            return self.get_icon_pixbuf(objtype.__bases__[0])


class Panel(gtk.Window):
    """\
    Control panel window.
    """
    class ControlBox(gtk.Frame):
        """\
        Object control box base class.
        """
        active_task = None

        def __init__(self, label, obj, command):
            super(Panel.ControlBox, self).__init__(label)
            self.obj = obj
            self.command = command
            self._update_mask = False

        def connect_entry(self, widget):
            for signal in ['activate', 'focus-out-event']:
                widget.connect(signal, self._set_data)

        def set_active_task(self, task):
            type(self).active_task = task

        def cleanup(self):
            pass

        def _update_data(self, widget, data=None):
            self.update_data()

        def _set_data(self, widget, data=None):
            if self._update_mask:
                return
            self.set_data()
            self.update_data()


    class PosableFrame(ControlBox):
        """\
        Object control frame for L{Posable} objects.
        """
        def __init__(self, obj, command):
            super(Panel.PosableFrame, self).__init__('Pose', obj, command)
            table = gtk.Table(3, 5)
            table.set_border_width(5)
            table.set_row_spacings(5)
            table.set_col_spacings(5)
            self.add(table)
            table.attach(gtk.Label('x'), 0, 1, 0, 1, xoptions=0)
            table.attach(gtk.Label('y'), 0, 1, 1, 2, xoptions=0)
            table.attach(gtk.Label('z'), 0, 1, 2, 3, xoptions=0)
            self.t = []
            for i in range(3):
                self.t.append(NumericEntry())
                self.t[-1].set_alignment(0.5)
                self.connect_entry(self.t[-1])
                table.attach(self.t[-1], 1, 2, i, i + 1)
            table.attach(gtk.Label(u'\u03b8'), 2, 3, 0, 1, xoptions=0)
            table.attach(gtk.Label(u'\u03c6'), 2, 3, 1, 2, xoptions=0)
            table.attach(gtk.Label(u'\u03c8'), 2, 3, 2, 3, xoptions=0)
            self.r = []
            for i in range(3):
                self.r.append(NumericEntry())
                self.r[-1].set_alignment(0.5)
                self.connect_entry(self.r[-1])
                table.attach(self.r[-1], 3, 4, i, i + 1)
            self.absolute = gtk.RadioButton(label='Absolute')
            self.absolute.connect('toggled', self._update_data)
            table.attach(self.absolute, 4, 5, 0, 1, xoptions=gtk.FILL)
            relative = gtk.RadioButton(group=self.absolute, label='Relative')
            table.attach(relative, 4, 5, 1, 2, xoptions=gtk.FILL)
            modifypose = gtk.ToggleButton('Modify')
            modifypose.connect('toggled', self._modify)
            table.attach(modifypose, 4, 5, 2, 3, xoptions=gtk.FILL)
            self.update_data()

        def update_data(self):
            self._update_mask = True
            if self.absolute.get_active():
                pose = self.command('getpose %s' % self.obj)
            else:
                pose = self.command('getrelativepose %s' % self.obj)
            for i in range(3):
                self.t[i].set_text(str(pose.T[i]))
            angles = pose.R.to_euler_zyx()
            for i in range(3):
                self.r[i].set_text(str(angles[i]))
            self._update_mask = False

        def set_data(self):
            pose = ' '.join([self.obj, 'euler-zyx-rad',
                ' '.join([self.t[i].get_text() for i in range(3)]),
                ' '.join([self.r[i].get_text() for i in range(3)])])
            if self.absolute.get_active():
                self.command('setpose %s' % pose)
            else:
                self.command('setrelativepose %s' % pose)

        def cleanup(self):
            self.command('modify')

        def _modify(self, widget, data=None):
            if widget.get_active():
                self.command('modify %s' % self.obj)
            else:
                self.command('modify')
                self.update_data()


    class CameraFrame(ControlBox):
        """\
        Object control frame for L{Camera} objects.
        """
        def __init__(self, obj, command):
            super(Panel.CameraFrame, self).__init__('Camera', obj, command)
            self.par = {}
            labels = {'f': 'f', 'A': 'A', 'zS': 'zS', 's': 's', 'o': 'o',
                      'dim': 'D'}
            table = gtk.Table(3, 6)
            table.set_border_width(5)
            table.set_row_spacings(5)
            table.set_col_spacings(5)
            self.add(table)
            for i, param in enumerate(['f', 'A', 'zS']):
                table.attach(gtk.Label(labels[param]), 0, 1, i, i + 1,
                    xoptions=0)
                self.par[param] = NumericEntry()
                self.par[param].set_alignment(0.5)
                self.connect_entry(self.par[param])
                table.attach(self.par[param], 1, 2, i, i + 1)
            for i, param in enumerate(['s', 'o', 'dim']):
                table.attach(gtk.Label(labels[param]), 2, 3, i, i + 1,
                    xoptions=0)
                self.par[param] = []
                for j in range(2):
                    self.par[param].append(NumericEntry())
                    self.par[param][-1].set_alignment(0.5)
                    self.connect_entry(self.par[param][-1])
                    table.attach(self.par[param][-1], 3 + j, 4 + j, i, i + 1)
            self.active = gtk.CheckButton('Active')
            self.active.connect('toggled', self._set_data)
            table.attach(self.active, 5, 6, 0, 1, xoptions=gtk.FILL)
            self.guide = gtk.ToggleButton('Frustum')
            self.guide.set_active((self.obj in self.command('activeguides')))
            self.guide.set_sensitive(True if self.active_task else False)
            self.guide.connect('toggled', self._guide)
            table.attach(self.guide, 5, 6, 2, 3, xoptions=gtk.FILL)
            self.update_data()

        def update_data(self):
            self._update_mask = True
            params = self.command('getparams %s' % self.obj)
            for param in self.par:
                if hasattr(params[param], '__iter__'):
                    for i in range(len(params[param])):
                        self.par[param][i].set_text(str(params[param][i]))
                else:
                    self.par[param].set_text(str(params[param]))
            self.active.set_active(self.command('getactive %s' % self.obj))
            self._update_mask = False

        def set_data(self):
            for param in self.par:
                if hasattr(self.par[param], '__iter__'):
                    value = ' '.join([p.get_text() for p in self.par[param]])
                else:
                    value = self.par[param].get_text()
                self.command('setparam %s %s %s' % (self.obj, param, value))
            if not self.active.get_active() == \
                self.command('getactive %s' % self.obj):
                self.command('setactive %s' % self.obj)
            if self.obj in self.command('activeguides'):
                self.command('guide %s' % self.obj)
                self.command('guide %s %s' % (self.obj, self.active_task))

        def set_active_task(self, task):
            super(Panel.CameraFrame, self).set_active_task(task)
            self.guide.set_sensitive(True if self.active_task else False)

        def _guide(self, widget, data=None):
            if self.active_task:
                self.command('guide %s %s' % (self.obj, self.active_task))


    class RobotFrame(ControlBox):
        """\
        Object control frame for L{Robot} objects.
        """
        def __init__(self, obj, command):
            super(Panel.RobotFrame, self).__init__('Robot', obj, command)
            joints = self.command('getjoints %s' % self.obj)
            table = gtk.Table(4, len(joints))
            table.set_border_width(5)
            table.set_row_spacings(5)
            table.set_col_spacings(5)
            self.add(table)
            self.joint = []
            for i, joint in enumerate(joints):
                table.attach(gtk.Label(joint['name']),
                    0, 1, i, i + 1, xoptions=0)
                image = gtk.Image()
                image.set_from_pixbuf(gtk.gdk.pixbuf_new_from_file_at_size(\
                    pkg_resources.resource_filename(__name__,
                    'resources/icons/' + joint['type'] + '.png'), 16, 16))
                table.attach(image, 1, 2, i, i + 1, xoptions=0)
                if joint['type'] == 'revolute':
                    inc = pi / 180.0
                elif joint['type'] == 'prismatic':
                    inc = (joint['limits'][1] - joint['limits'][0]) / 100.0
                self.joint.append(gtk.Adjustment(joint['home'],
                    joint['limits'][0], joint['limits'][1], inc, inc, 0.0))
                spin = gtk.SpinButton(self.joint[-1])
                spin.set_width_chars(8)
                spin.set_numeric(True)
                spin.set_digits(2)
                spin.set_alignment(0.5)
                table.attach(spin, 2, 3, i, i + 1, xoptions=0)
                slider = gtk.HScale(self.joint[-1])
                slider.set_draw_value(False)
                table.attach(slider, 3, 4, i, i + 1)
                self.joint[-1].connect('value-changed', self._set_data)
            self.update_data()

        def update_data(self):
            self._update_mask = True
            config = self.command('getposition %s' % self.obj)
            for i in range(len(config)):
                self.joint[i].set_value(config[i])
            self._update_mask = False

        def set_data(self):
            self.command('setposition %s ' % self.obj + \
                ' '.join(['%g' % joint.get_value() for joint in self.joint]))

        def _set_data(self, widget, data=None):
            if self._update_mask:
                return
            self.set_data()
            # no need to update robot data, too slow anyway


    class LineLaserFrame(ControlBox):
        """\
        Object control frame for L{LineLaser} objects.
        """
        def __init__(self, obj, command):
            super(Panel.LineLaserFrame, self).__init__('Line Laser', obj, command)
            self.par = {}
            labels = {'fan': u'\u03bb', 'depth': 'zP'}
            table = gtk.Table(1, 5)
            table.set_border_width(5)
            table.set_row_spacings(5)
            table.set_col_spacings(5)
            self.add(table)
            for i, param in enumerate(['fan', 'depth']):
                table.attach(gtk.Label(labels[param]), i * 2, i * 2 + 1, 0, 1,
                    xoptions=0)
                self.par[param] = NumericEntry()
                self.par[param].set_alignment(0.5)
                self.connect_entry(self.par[param])
                table.attach(self.par[param], i * 2 + 1, i * 2 + 2, 0, 1)
            self.guide = gtk.ToggleButton('Triangle')
            self.guide.set_active((self.obj in self.command('activeguides')))
            self.guide.connect('toggled', self._guide)
            table.attach(self.guide, 4, 5, 0, 1, xoptions=gtk.FILL)
            self.update_data()

        def update_data(self):
            self._update_mask = True
            params = self.command('getparams %s' % self.obj)
            for param in self.par:
                self.par[param].set_text(str(params[param]))
            self._update_mask = False

        def set_data(self):
            for param in self.par:
                value = self.par[param].get_text()
                self.command('setparam %s %s %s' % (self.obj, param, value))
            if self.obj in self.command('activeguides'):
                self.command('guide %s' % self.obj)
                self.command('guide %s' % self.obj)

        def _guide(self, widget, data=None):
            self.command('guide %s' % self.obj)


    class TaskFrame(ControlBox):
        """\
        Object control frame for L{Task} objects.
        """
        def __init__(self, obj, command):
            super(Panel.TaskFrame, self).__init__('Task', obj, command)
            self.par = {}
            labels = {'ocular': 'O', 'boundary_padding': u'\u03b3',
                      'res_min': u'R\u21a5', 'res_max': u'R\u21a7',
                      'blur_max': 'c', 'angle_max': u'\u03b6',
                      'hres_min': u'H\u21a5', 'inc_angle_max': u'\u03c9c'}
            params = self.command('getparams %s' % self.obj)
            range_task = 'inc_angle_max' in params
            table = gtk.Table(6 if range_task else 5, 5)
            table.set_border_width(5)
            table.set_row_spacings(5)
            table.set_col_spacings(5)
            self.add(table)
            for i, param in enumerate(['ocular', 'boundary_padding'] + \
                (['inc_angle_max'] if range_task else [])):
                table.attach(gtk.Label(labels[param]), 0, 1, i + 1, i + 2,
                    xoptions=0)
                self.par[param] = NumericEntry()
                self.par[param].set_alignment(0.5)
                self.connect_entry(self.par[param])
                table.attach(self.par[param], 1, 2, i + 1, i + 2)
            table.attach(gtk.Label('Ideal'), 3, 4, 0, 1)
            table.attach(gtk.Label('Accept'), 4, 5, 0, 1)
            for i, param in enumerate(['res_min', 'res_max', 'blur_max',
                'angle_max'] + (['hres_min'] if range_task else [])):
                table.attach(gtk.Label(labels[param]), 2, 3, i + 1, i + 2,
                    xoptions=0)
                self.par[param] = []
                for j in range(2):
                    self.par[param].append(NumericEntry())
                    self.par[param][-1].set_alignment(0.5)
                    self.connect_entry(self.par[param][-1])
                    table.attach(self.par[param][-1], 3 + j, 4 + j, i + 1, i + 2)
            self.update_data()

        def update_data(self):
            self._update_mask = True
            params = self.command('getparams %s' % self.obj)
            for param in self.par:
                if hasattr(params[param], '__iter__'):
                    for i in range(len(params[param])):
                        self.par[param][i].set_text(str(params[param][i]))
                else:
                    self.par[param].set_text(str(params[param]))
            self._update_mask = False

        def set_data(self):
            for param in self.par:
                if hasattr(self.par[param], '__iter__'):
                    value = ' '.join([p.get_text() for p in self.par[param]])
                else:
                    value = self.par[param].get_text()
                self.command('setparam %s %s %s' % (self.obj, param, value))
            # refresh guides on task change
            if self.active_task == self.obj:
                for key in self.command('activeguides'):
                    self.command('guide %s' % key)
                    self.command('guide %s %s' % (key, self.obj))


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
        vbox.set_spacing(5)
        vbox.set_border_width(5)
        self.add(vbox)

        # menu bar
        menubar = gtk.MenuBar()
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
        menuc = gtk.MenuItem('Coverage')
        menu = gtk.Menu()
        self.cov = {}
        self.cov['standard'] = gtk.MenuItem('Standard')
        self.cov['standard'].connect('activate', self._coverage_standard)
        self.cov['standard'].set_sensitive(False)
        menu.add(self.cov['standard'])
        menu.add(gtk.SeparatorMenuItem())
        self.cov['lr'] = gtk.MenuItem('Range (Linear)')
        #self.cov['lr'].connect('activate', self._coverage_range_linear)
        self.cov['lr'].set_sensitive(False)
        menu.add(self.cov['lr'])
        self.cov['rr'] = gtk.MenuItem('Range (Rotary)')
        #self.cov['rr'].connect('activate', self._coverage_range_rotary)
        self.cov['rr'].set_sensitive(False)
        menu.add(self.cov['rr'])
        menuc.set_submenu(menu)
        menubar.add(menuc)

        # fixed controls
        hbox = gtk.HBox()
        hbox.set_spacing(5)
        hbox.pack_start(gtk.Label('Task:'), expand=False)
        self.tasklist = gtk.combo_box_new_text()
        self.tasklist.connect('changed', self._select_task)
        hbox.pack_start(self.tasklist)
        vbox.pack_start(hbox, expand=False)

        # paned panel boxes
        vpaned = gtk.VPaned()
        vbox.pack_start(vpaned, True, True)
        sw = gtk.ScrolledWindow()
        sw.set_policy(gtk.POLICY_AUTOMATIC, gtk.POLICY_ALWAYS)
        sw.set_shadow_type(gtk.SHADOW_IN)
        self.objecttreeview = ObjectTreeView()
        self.objecttreeview.get_selection().connect('changed',
            self._select_object)
        sw.add(self.objecttreeview)
        vpaned.add1(sw)
        sw = gtk.ScrolledWindow()
        sw.set_policy(gtk.POLICY_NEVER, gtk.POLICY_ALWAYS)
        self.controlbox = gtk.VBox()
        self.controlbox.set_spacing(5)
        self.controlbox.set_border_width(5)
        sw.add_with_viewport(self.controlbox)
        vpaned.add2(sw)

        # show panel
        self.show_all()

        # populate object tree
        self.populate_object_tree()
        self.populate_task_list()

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
        self.hierarchy = self.ad_command('objecthierarchy')
        self.objecttreeview.populate(copy(self.hierarchy))

    def populate_task_list(self):
        self.tasklist.get_model().clear()
        tasks = self.ad_command('tasks')
        for task in tasks:
            self.tasklist.append_text(task)

    def _delete_event(self, widget, data=None):
        return False

    def _destroy(self, widget, data=None):
        self.ad_command('exit')
        gtk.main_quit()

    def _update_data(self, widget, data=None):
        for child in self.controlbox.get_children():
            child.update_data()

    def _set_data(self, widget, data=None):
        for child in self.controlbox.get_children():
            child.set_data()

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
            self.populate_task_list()
        dialog.destroy()

    def _select_object(self, widget, data=None):
        model, selected = widget.get_selected_rows()
        for child in self.controlbox.get_children():
            child.cleanup()
            self.controlbox.remove(child)
        try:
            obj = model.get_value(model.get_iter(selected[0]), 1)
            objclass = self.hierarchy[obj][1]
            bases = [objclass]
            while bases:
                newbases = []
                for base in bases:
                    try:
                        self.controlbox.pack_start(getattr(self, base.__name__ \
                            + 'Frame')(obj, self.ad_command), expand=False)
                    except AttributeError:
                        pass
                    for newbase in base.__bases__:
                        newbases.append(newbase)
                bases = newbases
            self.controlbox.show_all()
            if issubclass(objclass, Task):
                self.ad_command('select')
            else:
                self.ad_command('select %s' % obj)
            for child in self.controlbox.get_children():
                child.set_active_task(self.get_active_task())
        except IndexError:
            self.ad_command('select')

    def get_active_task(self):
        selected = self.tasklist.get_active()
        model = self.tasklist.get_model()
        if selected < 0:
            return None
        else:
            return model.get_value(model.get_iter(selected), 0)

    def _select_task(self, widget, data=None):
        task = self.get_active_task()
        for child in self.controlbox.get_children():
            child.set_active_task(task)
        if task is None:
            for key in self.cov:
                self.cov[key].set_sensitive(False)
        else:
            self.cov['standard'].set_sensitive(True)
            rt = issubclass(self.hierarchy[task][1], RangeTask)
            self.cov['lr'].set_sensitive(rt)
            self.cov['rr'].set_sensitive(rt)

    def _setcenter(self, widget, data=None):
        c = self.ad_command('getcenter')
        dialog = gtk.Dialog('Set Center', self, 0, (gtk.STOCK_OK,
            gtk.RESPONSE_OK, gtk.STOCK_CANCEL, gtk.RESPONSE_CANCEL))
        dialog.set_border_width(5)
        table = gtk.Table(2, 3)
        table.set_row_spacings(5)
        table.set_col_spacings(5)
        dialog.vbox.pack_start(table, True, True, 0)
        table.attach(gtk.Label('x'), 0, 1, 1, 2)
        table.attach(gtk.Label('y'), 1, 2, 1, 2)
        table.attach(gtk.Label('z'), 2, 3, 1, 2)
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

    def _coverage_standard(self, widget, data=None):
        selected = self.tasklist.get_active()
        model = self.tasklist.get_model()
        task = model.get_value(model.get_iter(selected), 0)
        self.ad_command('coverage %s' % task)
