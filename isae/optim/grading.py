class grading_RMS:
	def __init__(self):
		self.grade_total = 0

	def grade(self, q, qdot, qdot_ref, dt):
		for i in range(len(qdot_ref)):
			if(qdot_ref[i] != None):
				self.grade_total -= (dt ** 2) * (qdot[i][0] - qdot_ref[i][0]) ** 2

	def getGrade(self):
		return self.grade_total