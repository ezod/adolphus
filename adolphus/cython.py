"""\
Cython import setup module.

Adapted from http://wiki.cython.org/InstallingOnWindows.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import os
import pyximport


if os.name == 'nt':
    if os.path.exists('C:\MinGW'):
        try:
            os.environ['PATH'] += ';C:\MinGW\bin'
        except KeyError:
            os.environ['PATH'] = 'C:\MinGW\bin'
        mingw_setup_args = {'options': {'build_ext': {'compiler': 'mingw32'}}}
        pyximport.install(setup_args=mingw_setup_args)
    else:
        pyximport.install()
elif os.name == 'posix':
    pyximport.install()
