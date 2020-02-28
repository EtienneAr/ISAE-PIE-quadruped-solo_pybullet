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
		self.Time = 0

	def c(self, q, q_dot, time, dt):
		self.Time += dt

		if(self.Time > self.period):
			self.Time = 0

			self.positionBias = (np.random.rand(8) - 0.5) * 2. * self.positionNoise
			self.velocityBias = (np.random.rand(8) - 0.5) * 2. * self.velocityNoise
			self.torqueBias = np.random.rand(8) - 0.5 * 2. * self.torqueNoise
			
		q_biased     = np.vstack((q[:7],     q[7:]     + np.matrix(self.positionBias).T))
		q_dot_biased = np.vstack((q_dot[:6], q_dot[6:] + np.matrix(self.velocityBias).T))

		torque = self.controller.c(q_biased, q_dot_biased, time, dt)

		return torque + np.matrix(self.torqueBias).T
