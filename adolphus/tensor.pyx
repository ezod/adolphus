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
	def __cinit__(self, m=[]):
		"""\
		Constructor.

		@param m: The n x m tensor matrix.
        @type m: C{list} of C{list}
		"""
		self._m = m
		self._h = len(m)
		if self._h == 0:
			self._w = 0
			vector = []
		else:
			try:
				self._w = len(m[0])
				if self._w == 0:
					self._h = 0
					vector = []
				else:
					vector = []
					for row in m:
						assert len(row) == self._w, "All rows must have the same size."
						for col in row:
							assert isinstance(col, float) or isinstance(col, int), \
								"No valid type cast exists from " + \
								type(col).__name__ + " to \'float\'."
							vector.append(col)
			except TypeError:
				self._w = 1
				for col in m:
					assert isinstance(col, float) or isinstance(col, int), \
						"No valid type cast exists from " + \
						type(col).__name__ + " to \'float\'."
				vector = m

		self._tensor = array('d', vector)

	def __hash__(self):
		return hash(repr(self))

	def __reduce__(self):
		return (Tensor, (self._m))

	def __getitem__(self, ij):
		i,j = ij
		return self._tensor.__getitem__((i * self._w) + j)

	def __richcmp__(self, Tensor t, int o):
		h1, w1 = self.size
		h2, w2 = t.size
		assert h1 == h2 and w1 == w2, "Both tensors must be of the same size."
		cdef bool eq = True
		for i in xrange(h1):
			for j in xrange(w2):
				if abs(self[i,j] - t[i,j]) > 1e-4:
					eq = False
					break
		if o == 2:
			return eq
		if o == 3:
			return not eq

	def __repr__(self):
		"""\
		Canonical string representation.
		"""
		line = type(self).__name__ + "(["
		for i in xrange(self._h):
			line += "["
			for j in xrange(self._w):
				if j < self._h - 1:
					line += str(self._tensor.__getitem__((i * self._w) + j)) + ", "
				elif j == self._h - 1:
					line += str(self._tensor.__getitem__((i * self._w) + j))
			if i < self._w - 1:
				line += "],\n        "
			elif i == self._w - 1:
				line += "]"
		line += "])"
		return line

	def __str__(self):
		"""\
		String representation, displays in a tuple format.
		"""
		line = "(["
		for i in xrange(self._h):
			line += "["
			for j in xrange(self._w):
				if j < self._h - 1:
					line += str(self._tensor.__getitem__((i * self._w) + j)) + ", "
				elif j == self._h - 1:
					line += str(self._tensor.__getitem__((i * self._w) + j))
			if i < self._w - 1:
				line += "],\n  "
			elif i == self._w - 1:
				line += "]"
		line += "])"
		return line

	@property
	def size(self):
		"""\
		Return a tuple containing the width and height of the Tensor.
		"""
		return (self._h, self._w)
