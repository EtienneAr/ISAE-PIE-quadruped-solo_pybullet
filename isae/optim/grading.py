import numpy as np

class grading_RMS:
	def __init__(self, qdot_ref, factors):
		self.qdot_ref = qdot_ref[:]
		self.factors = factors[:]
		self.grade_total = 0

	def updateGrade(self, q, qdot, time, dt):
		self.grade_total -= np.sum(self.factors * (qdot[:6] - self.qdot_ref) ** 2) * dt

	def getGrade(self):
		return self.grade_total