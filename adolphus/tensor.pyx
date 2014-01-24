"""\
Tensor module. Contains Tensor classes and a few operations such as a distance function
between tensors. The base Tensor class is implemented as an m x n matrix.

@author: Jose Alarcon
@organization: University of Windsor
@contact: alarconj@uwindsor.ca
@license: GPL-3
"""

from array import array
from cpython cimport bool

cdef class Tensor:
	"""\
	Tensor class.
	"""
	def __cinit__(self, v=[]):
		"""\
		Constructor.
		"""
		self._h = len(v)
		if self._h == 0:
			self._w = 0
			vector = []
		else:
			try:
				self._w = len(v[0])
				if self._w == 0:
					self._h = 0
					vector = []
				else:
					vector = []
					for row in v:
						assert len(row) == self._w, "All rows must have the same size."
						for col in row:
							assert isinstance(col, float) or isinstance(col, int), \
								"No valid type cast exists from " + \
								str(type(col))[1:-1] + " to type \'float\'."
							vector.append(col)
			except TypeError:
				self._w = 1
				for col in v:
					assert isinstance(col, float) or isinstance(col, int), \
						"No valid type cast exists from " + str(type(col))[1:-1] + \
						" to type \'float\'."
				vector = v

		self._tensor = array('d', vector)

	@property
	def size(self):
		"""\
		Return a tuple containing the width and height of the Tensor.
		"""
		return (self._h, self._w)
