class pointsTrajectory:
	def __init__(self, points, factor = [1,1]):
		self.factor = factor
		self.points = [[-1, 0]] + points + [[1, 0]]

	def getPos(self, phase, factor = None):
		factorTotal = self.factor
		if(factor != None):
			for i in range(len(factor)):
				factorTotal *= factor[i]
		
		#Contact with the ground
		phase %= 1

		if(phase < 0.5):
			x_pos = factorTotal[0] * (0.5 - phase*2.)
			y_pos = 0

			return [x_pos, y_pos]
	
		#Navigates between points
		sub_phase = 2. * (phase - 0.5)
		
		current_point = min(int(sub_phase * (len(self.points)-1)), len(self.points)-2)
		sub_phase_for_point = sub_phase  * (len(self.points)-1) - 1.0 * current_point

		prev_point = self.points[current_point]
		next_point = self.points[current_point+1]

		x_pos = prev_point[0] + (next_point[0]-prev_point[0]) * sub_phase_for_point
		y_pos = prev_point[1] + (next_point[1]-prev_point[1]) * sub_phase_for_point
		
		return [x_pos * factorTotal[0] /2., y_pos * factorTotal[1]]