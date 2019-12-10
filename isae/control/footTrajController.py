from sys import path
import numpy as np
from isae.control.myPD import PD

class footTrajController:

	# initialization
	def __init__(self, bodyHeights, Leg, sols, Feet4traj,  period, Kp = 0, Kd = 0, sat = float('inf')):
		self.Kp = Kp*1. # 8.
		self.Kd = Kd*1. # 0.2
		self.sat = sat  # 3 # saturation of the controller
		self.Feet4traj = Feet4traj # array of continuousTraj, length 4
		self.sols = sols # array of boolean, length 4, chooses which IK solution to chose
		self.Leg = Leg # leg geometry
		self.bHs = bodyHeights # in the trajectory ref frame
		self.period = period # period of the trajectory cycle

		#init
		self.currentPhase = 0.

	# called every step of simulation by __main__
	def c(self, q, q_dot, time, dt): 
		# keeps track of "pos" (moment) in the cycle
		self.currentPhase += dt / self.period
		# get 4 [x,y] positions from the Feet4traj array of trajectories
		traj_pos_ref = map(lambda contTraj : contTraj.getPos(self.currentPhase), self.Feet4traj)
		# remap the positions in leg frame
		#legs_pos_ref = map(lambda pos : [pos[0,0],pos[0,1]-self.bH], traj_pos_ref)
		legs_pos_ref = [[traj_pos_ref[i][0,0],traj_pos_ref[i][0,1]-self.bHs[i]] for i in range(len(traj_pos_ref))]
		# compute desired joints positions [theta0, theta1] for each leg
		#q_ref_temp = map(lambda pos : self.Leg.getJointsPos(pos, otherSol = False), legs_pos_ref)
		q_ref_temp = [self.Leg.getJointsPos(legs_pos_ref[i], otherSol = self.sols[i]) for i in range(4)]

		# remap q_ref as array of float instead array of pairs 
		q_ref = []
		for qq in q_ref_temp:
			q_ref.append([qq[0]])
			q_ref.append([qq[1]])
		
		# compute desired joints torques
		torques = PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)

		return torques