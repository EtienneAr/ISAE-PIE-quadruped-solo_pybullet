from sys import path
import numpy as np
#import _thread
from isae.myPD import PD

def async_input(controller):
	while(True):
		fact = float(input("New speed factor : "))
		controller.factor = [fact, abs(fact)]

class myController:
	def __init__(self, bodyHeight, Leg, FootTraj, period, phasesOffset, Kp = 0, Kd = 0, sat = float('inf'), userInput = False):
		self.Kp = Kp*1. # 8.
		self.Kd = Kd*1. # 0.2
		self.sat = sat  # 3
		self.FootTraj = FootTraj # type: pointsTrajectory in trajectory.py
		self.Leg = Leg # type: Leg in geometry.py
		self.bH = bodyHeight
		self.phasesOffset = phasesOffset
		self.period = period

		#init
		self.currentPhase = 0.
		self.factor = [1, 1]
		#if(userInput):
		#	_thread.start_new_thread(async_input, (self,))

	def c(self, q, q_dot, time, dt):
		self.currentPhase += dt / self.period
		pos_ref = map(lambda phase : self.FootTraj.getPos(phase + self.currentPhase, self.factor), self.phasesOffset)
		joints_pos_ref = map(lambda pos : [pos[0],pos[1]-self.bH], pos_ref)
		q_ref_temp = map(lambda joints_pos : self.Leg.getJointsPos(joints_pos), joints_pos_ref)

		q_ref = []
		for qq in q_ref_temp:
			q_ref.append([qq[0]])
			q_ref.append([qq[1]])

		torques = PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)

		return torques
