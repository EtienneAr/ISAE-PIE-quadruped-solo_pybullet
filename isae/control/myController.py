from sys import path
import numpy as np
import _thread
import isae.control.myPD

def async_input(controller):
	while(True):
		fact = float(input("New speed factor : "))
		controller.factor = [fact, abs(fact)]

class myController:
	def __init__(self, bodyHeight, Leg, FootTraj, period, phasesOffset, Kp = 0, Kd = 0, sat = float('inf'), userInput = False):
		self.Kp = Kp*1. # 8.
		self.Kd = Kd*1. # 0.2
		self.sat = sat  # 3
		self.FootTraj = FootTraj
		self.Leg = Leg
		self.bH = bodyHeight
		self.phasesOffset = phasesOffset
		self.period = period

		#init
		self.currentPhase = 0.
		self.factor = [1, 1]
		if(userInput):
			#create an asynchronous input to allow user to change paramters while the simulation is running
			# by entering them in the terminal
			_thread.start_new_thread(async_input, (self,))

	def c(self, q, q_dot, time, dt):
		self.currentPhase += dt / self.period
		pos_ref = map(lambda phase : self.FootTraj.getPos(phase + self.currentPhase, self.factor), self.phasesOffset)
		pos_ref = map(lambda pos : [pos[0],pos[1]-self.bH], pos_ref)
		q_ref_temp = map(lambda pos : self.Leg.getJointsPos(pos), pos_ref)

		q_ref = []
		for qq in q_ref_temp:
			q_ref.append([qq[0]])
			q_ref.append([qq[1]])

		torques = isae.control.myPD.PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)

		return torques
