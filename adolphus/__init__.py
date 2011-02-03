"""\
Coverage module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

__version__ = (0, 0, 0)

__all__ = ['geometry',
           'posable',
           'graph',
           'coverage',
           'interface',
           'visualization',
           'yamlparser']
__name__ = 'adolphus'

import pyximport; pyximport.install()

from geometry import *
from posable import *
from graph import *
from coverage import *
from interface import *
from visualization import *
from yamlparser import *
