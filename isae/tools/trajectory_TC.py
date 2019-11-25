class pointsTrajectory:
	def __init__(self, points, factor = [1,1]):
		self.factor = factor
		self.points = points  

	#Compute x,y : Bezier poly with 4 control points, t € [0,1]
	def point_bezier_3(self,points_control,t):
		x=(1-t)**2
		y=t*t
		A = self.combinaison_lineaire(points_control[0],points_control[1],(1-t)*x,3*t*x)
		B = self.combinaison_lineaire(points_control[2],points_control[3],3*y*(1-t),y*t)
		
		return [A[0]+B[0],A[1]+B[1]]

	def combinaison_lineaire(self,A,B,u,v):
		return [A[0]*u+B[0]*v,A[1]*u+B[1]*v]
		

	def getPos(self, phase, factor = None):
		factorTotal = self.factor
		if(factor != None):
			for i in range(len(factor)):
				factorTotal *= factor[i]
		
		#Contact with the ground
		phase %= 1

		#Curve separated into segments with 4 control points
		#Get index of the segment
		nb_curve = (len(self.points)-1)//3
		a = 1/nb_curve
		b = phase // a + 1
		indice_1 = int(b*3 - 3)

		#Get time inside the segment, t € [0,1]
		t_ref_poly = (phase % a )*nb_curve	

		points_control = [self.points[indice_1 ], self.points[indice_1 + 1] ,self.points[indice_1 + 2] , self.points[indice_1+ 3] ]
		 
		return self.point_bezier_3(points_control, t_ref_poly)

	