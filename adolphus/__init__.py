"""\
Adolphus - Multi-Camera System Coverage Modeling Suite

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
__version__ = (0, 1, 0)
