import numpy as np

class noiseIn_noiseOut:
	def __init__(self, controller, period, positionNoise=0, velocityNoise=0, torqueNoise=0):
		self.controller = controller
		self.positionNoise = positionNoise
		self.velocityNoise = velocityNoise
		self.torqueNoise   = torqueNoise

		self.positionBias = np.array([0] * 8)
		self.velocityBias = np.array([0] * 8)
		self.torqueBias = np.array([0] * 8)

		self.period = period
		self.lastTime = -1e6

	def c(self, q, q_dot, time, dt):
		if(time > self.lastTime + self.period):
			self.positionBias = (np.random.rand(8) - 0.5) * 2. * self.positionNoise
			self.velocityBias = (np.random.rand(8) - 0.5) * 2. * self.velocityNoise
			self.torqueBias = np.random.rand(8) - 0.5 * 2. * self.torqueNoise

			self.lastTime = time
			
		q_biased     = np.vstack((q[:7],     q[7:]     + np.matrix(self.positionBias).T))
		q_dot_biased = np.vstack((q_dot[:6], q_dot[6:] + np.matrix(self.velocityBias).T))

		torque = self.controller.c(q_biased, q_dot_biased, time, dt)

		# print()
		# print(q)
		# print(q_dot)
		return torque + np.matrix(self.torqueBias).T
