from sys import path
import numpy as np
#import _thread
from isae.control.myPD import PD

def async_input(controller):
	while(True):
		fact = float(input("New speed factor : "))
		controller.factor = [fact, abs(fact)]

class footTrajController:
	def __init__(self, bodyHeight, Legs, FootTraj, period, phasesOffset, Kp = 0, Kd = 0, sat = float('inf')):
		self.Kp = Kp*1. # 8.
		self.Kd = Kd*1. # 0.2
		self.sat = sat  # 3
		self.FootTraj = FootTraj
		self.Legs = Legs
		self.bH = bodyHeight
		self.phasesOffset = phasesOffset
		self.period = period

		#init
		self.currentPhase = 0.

		self.segLengths = (np.diff(FootTraj.points[:,0])**2 + np.diff(FootTraj.points[:,1])**2)**.5 # segment lengths, size n
		self.cumulLength = np.concatenate(([0],np.cumsum(self.segLengths))) # cumulated lengths, size n

	def c(self, q, q_dot, time, dt):
		self.currentPhase += dt / self.period
		pos_ref = map(lambda phase : self.FootTraj.getPos(phase + self.currentPhase, self.cumulLength), self.phasesOffset)
		pos_ref = map(lambda pos : [pos[0,0],pos[0,1]-self.bH], pos_ref)
		q_ref_temp = map(lambda pos : self.Legs.getJointsPos(pos), pos_ref)

		q_ref = []
		for qq in q_ref_temp:
			q_ref.append([qq[0]])
			q_ref.append([qq[1]])

		torques = PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)

		return torques