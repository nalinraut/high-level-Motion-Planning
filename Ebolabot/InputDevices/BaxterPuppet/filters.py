#Jordan Pierce

from klampt import vectorops

#just a filter I did myself, more for practice than anything else
class AveragesFilter:
	def __init__(self):
		self.num_runs = 0
		self.dbl_prev_iter = []
		self.prev_iter = []
		self.this_iter = []
		self.result = []

	def process(self,inputs):
		self.result = []
		if self.num_runs < 3:
			self.num_runs += 1
		if self.num_runs == 1:
			self.dbl_prev_iter = inputs
			self.prev_iter = inputs
			self.this_iter = inputs
		elif self.num_runs == 2:
			self.dbl_prev_iter = inputs
			self.this_iter = inputs
		else:
			self.this_iter = inputs
		for i in zip(self.dbl_prev_iter, self.prev_iter, self.this_iter):
			self.result.append(sum(i)/len(i))
		self.dbl_prev_iter = self.prev_iter
		self.prev_iter = self.this_iter
		return self.result




class ExponentialFilter:
	def __init__(self, rate = 0.5):
		self.rate = rate
		self.value = None

	def process(self, inputs):
		if self.value == None or len(self.value) != len(inputs):
			self.value = inputs
		else:
			x = vectorops.mul(self.value, 1 - self.rate)
			self.value = vectorops.madd(x,inputs,self.rate)
		return self.value
		
class DeadbandFilter:

	def __init__(self, width = 0.1):
		self.width = width
		self.last = None

	def process(self, inputs):
		if self.last == None or len(self.last) != len(inputs):
			self.last = inputs[:]
		for i, (l, v) in enumerate(zip(self.last,inputs)):
			if abs(l - v) > self.width:
				self.last[i] = v
		return self.last




#a class that combines the effects of multiple filters
class CompositeFilter:

	def __init__(self, filters):
		self.filters = filters

	def process(self, inputs):
		vals = inputs
		for f in self.filters:
			vals = f.process(vals)
		return vals

