"""\
Coverage module.

@author: Aaron Mavrinac
@organization: University of Windsor
@contact: mavrin1@uwindsor.ca
@license: GPL-3
"""

import sys
import os.path

module = os.path.join(os.path.dirname(__file__), '../hypergraph')
if os.path.exists(module):
    sys.path.append(module)

__import__('pkg_resources').declare_namespace(__name__)
__version__ = (0, 0, 0)

import cython
from .geometry import *
from .posable import *
from .coverage import *
from .interface import *
from .visualization import *
from .yamlparser import *

try:
    from .panel import *
except ImportError:
    pass
