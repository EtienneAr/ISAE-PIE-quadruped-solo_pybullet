from sys import path
import numpy as np
#import _thread
from isae.control.myPD import PD

def async_input(controller):
	while(True):
		fact = float(input("New speed factor : "))
		controller.factor = [fact, abs(fact)]

class footTrajController:
	def __init__(self, bodyHeight, Leg, Feet4traj, period, Kp = 0, Kd = 0, sat = float('inf')):
		self.Kp = Kp*1. # 8.
		self.Kd = Kd*1. # 0.2
		self.sat = sat  # 3
		self.Feet4traj = Feet4traj # array of continuousTraj, length 4
		self.Leg = Leg # leg geometry
		self.bH = bodyHeight # in the trajectory ref frame
		self.period = period # period of the trajectory cycle

		#init
		self.currentPhase = 0.

	def c(self, q, q_dot, time, dt):
		self.currentPhase += dt / self.period
		#pos_ref = map(lambda phase : self.Feet4traj.getPos(phase + self.currentPhase), self.phasesOffset)
		pos_ref = map(lambda contTraj : contTraj.getPos(self.currentPhase), self.Feet4traj)
		pos_ref = map(lambda pos : [pos[0,0],pos[0,1]-self.bH], pos_ref)
		q_ref_temp = map(lambda pos : self.Leg.getJointsPos(pos, otherSol = False), pos_ref)

		q_ref = []
		for qq in q_ref_temp:
			q_ref.append([qq[0]])
			q_ref.append([qq[1]])

		torques = PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)

		return torques