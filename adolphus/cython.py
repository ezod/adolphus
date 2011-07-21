"""\
Cython import setup module.

Adapted from http://wiki.cython.org/InstallingOnWindows.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import numpy
import pyximport


if os.name == 'nt':
    try:
        os.environ['CPATH'] += numpy.get_include()
    except KeyError:
        os.environ['CPATH'] = numpy.get_include()
    try:
        os.environ['PATH'] += ';C:\MinGW\bin'
    except KeyError:
        os.environ['PATH'] = 'C:\MinGW\bin'
    mingw_setup_args = {'options': {'build_ext': {'compiler': 'mingw32'}}}
    pyximport.install(setup_args=mingw_setup_args)
elif os.name == 'posix':
    try:
        os.environ['CFLAGS'] += ' -I' + numpy.get_include()
    except KeyError:
        os.environ['CFLAGS'] = ' -I' + numpy.get_include()
    pyximport.install()
