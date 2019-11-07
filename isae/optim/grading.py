import numpy as np

class grading_RMS:
	def __init__(self):
		self.grade_total = 0

	def grade(self, q, qdot, qdot_ref, factors, dt):
		self.grade_total -= np.sum(factors * (qdot - qdot_ref) ** 2) * dt

	def getGrade(self):
		return self.grade_total