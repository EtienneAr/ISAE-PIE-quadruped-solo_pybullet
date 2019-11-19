from math import pi, atan, atan2, cos, sin, sqrt, pow


class Leg:
	def __init__(self, upperLegLen, lowerLegLen):
		self.l1 = upperLegLen*1.
		self.l2 = lowerLegLen*1.

	def getFootPos(self, jointsPos):
		return [self.l1*cos(jointsPos[0]- pi/2.) + self.l2 * cos(jointsPos[0]- pi/2.+jointsPos[1]),
				self.l2*sin(jointsPos[0]- pi/2.) + self.l2 * sin(jointsPos[0]- pi/2.+jointsPos[1])]

	def getJointsPos(self, footPos, otherSol = False):
		r2 = pow(footPos[0],2) + pow(footPos[1],2)
		theta_mean = atan2(footPos[1],footPos[0])
		d_theta_1 = 2*atan((self.l2+self.l1-sqrt(r2))/(self.l2-self.l1-sqrt(r2)) * sqrt((r2 - pow(self.l1-self.l2,2))/(pow(self.l1+self.l2,2)-r2)))

		if(otherSol):
			return [theta_mean + d_theta_1 + pi/2., -(pi + 2*atan(sqrt((r2 - pow(self.l1-self.l2,2)) / (pow(self.l1+self.l2,2)-r2))))]

		return [theta_mean - d_theta_1 + pi/2., pi + 2*atan(sqrt((r2 - pow(self.l1-self.l2,2)) / (pow(self.l1+self.l2,2)-r2)))]

	def getJointsTorque(self, jointsPos, load):
		#rGives the torque at each joint of a leg for a force applied at the shoulder
		kneePos = [self.l1*cos(jointsPos[0]), self.l1*sin(jointsPos[0])]
		footPos = self.getFootPos(jointsPos) 
		footForce = [-load[0], -load[1]]

		return [      footPos[0]       *footForce[1] -       footPos[1]       *footForce[0], 
				(footPos[0]-kneePos[0])*footForce[1] - (footPos[1]-kneePos[1])*footForce[0]]