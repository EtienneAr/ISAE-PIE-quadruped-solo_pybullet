class pointsTrajectory:
<<<<<<< Updated upstream
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

	
=======
	
    def __init__(self, points, factor = [1,1], pointsL = []):
        self.factor = factor
        self.points = points # form : [[x1 y1 x2 y2] , [delta_x1 d_y1 d_x2 d_y2]]  
                             # d_x1 : delta x for derivative to positionnate points around x1 
							 # --> assure continuous fonction
        self.pointsL = []

    #Compute x,y : Bezier poly with 4 control points, t € [0,1]
    def point_bezier_3(self,points_control,t):
        x=(1-t)**2
        y=t*t
        A = self.combinaison_lineaire(points_control[0],points_control[1],(1-t)*x,3*t*x)
        B = self.combinaison_lineaire(points_control[2],points_control[3],3*y*(1-t),y*t)

        return [A[0]+B[0],A[1]+B[1]]

    def combinaison_lineaire(self,A,B,u,v):
        return [A[0]*u+B[0]*v,A[1]*u+B[1]*v]
	
    def derivToList(self,derivList):
		#transform a list of points and derivative into list of points
        List_pt = []
        params_d = derivList[1]
        pts = derivList[0]          

        for k in range(len(pts)//2) :   
            P0_x = pts[2*k]
            P0_y = pts[2*k+1]
            P1_x = P0_x + params_d[2*k]
            P1_y = P0_y + params_d[2*k+1]
            if(2*k+3>len(pts)):
                P3_x = pts[0]
                P3_y = pts[1]
                P2_x = P3_x - params_d[0]
                P2_y = P3_y - params_d[1]
            else:
                P3_x = pts[2*k+2]
                P3_y = pts[2*k+3]
                P2_x = P3_x - params_d[2*k+2]
                P2_y = P3_y - params_d[2*k+3]
            List_pt.append([P0_x,P0_y])
            List_pt.append([P1_x,P1_y])
            List_pt.append([P2_x,P2_y])
        P0_x = pts[0]
        P0_y = pts[1]
        List_pt.append([P0_x,P0_y])
			
        return List_pt


    def getPos(self, phase, factor = None):
        if(len(self.pointsL) == 0):
            self.pointsL = self.derivToList(self.points)

        factorTotal = self.factor
        if(factor != None):
            for i in range(len(factor)):
                factorTotal *= factor[i]

        #Contact with the ground
        phase %= 1

		#Curve separated into segments with 4 control points
        #Get index of the segment
        nb_curve = (len(self.pointsL)-1)//3
        a = 1/nb_curve
        b = phase // a + 1
        indice_1 = int(b*3 - 3)

        #Get time inside the segment, t € [0,1]
        t_ref_poly = (phase % a )*nb_curve	

        points_control = [self.pointsL[indice_1 ], self.pointsL[indice_1 + 1] ,self.pointsL[indice_1 + 2] , self.pointsL[indice_1+ 3] ]

        return self.point_bezier_3(points_control, t_ref_poly)
	

    



>>>>>>> Stashed changes
