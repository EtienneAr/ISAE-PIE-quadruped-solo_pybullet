import math
import numpy as np
import scipy.optimize as op

class lerpCyclePhasePoly:

    def __init__(self, points_control):
        self.P0 = points_control[0]
        self.P1 = points_control[1]
        self.P2 = points_control[2]
        self.P3 = points_control[3]
        self.x0 = 0

    def combinaison_lineaire(self,A,B,u,v):
        return [A[0]*u+B[0]*v,A[1]*u+B[1]*v]

    def point_bezier_3(self,t):
        x=(1-t)**2
        y=t*t
        A = self.combinaison_lineaire(self.P0,self.P1,(1-t)*x,3*t*x)
        B = self.combinaison_lineaire(self.P2,self.P3,3*y*(1-t),y*t)
        return [A[0]+B[0],A[1]+B[1]]


    def f(self,t):
        x=(1-t)**2
        y=t*t
        A = self.combinaison_lineaire(self.P0,self.P1,(1-t)*x,3*t*x)
        B = self.combinaison_lineaire(self.P2,self.P3,3*y*(1-t),y*t)
        return A[0]+B[0] - self.x0

    def lerpCyclePhase_3(self,t):
		#polynomial of degree 3
        self.x0 = 0
        return self.point_bezier_3(t)[0]

    def lerpCyclePhase_bezier(self,t):
		#long execution time, but allows to explore a larger space
        self.x0 = t
        t1 = op.fsolve(self.f,self.x0)
        return self.point_bezier_3(t1)[1][0]