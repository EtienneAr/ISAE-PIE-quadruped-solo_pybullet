import numpy as np

class footTrajectoryBezier:
	
    def __init__(self, points, phaseOffset = 0, ratio = [0,0]):
        self.points = points # form : [[x1 y1 x2 y2] , [delta_x1 d_y1 d_x2 d_y2]]  
                             # d_x1 : delta x for derivative to positionnate points around x1 
							 # --> assure continuous fonction
        self.pointsL = []
        self.phaseOffset = phaseOffset
        self.ratio = ratio

    #Compute x,y : Bezier poly with 4 control points, t € [0,1]
    def point_bezier_3(self,points_control,t):
        x=(1-t)**2
        y=t*t
        A = self.combinaison_lineaire(points_control[0],points_control[1],(1-t)*x,3*t*x)
        B = self.combinaison_lineaire(points_control[2],points_control[3],3*y*(1-t),y*t)

        return np.array([[A[0]+B[0],A[1]+B[1]]])

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


    def getPos(self, phase):
        if(len(self.pointsL) == 0):
            self.pointsL = self.derivToList(self.points)

        #Contact with the ground
        phase = (phase + self.phaseOffset) %1

        phase = adaptMieux(phase) % 1

		#Curve separated into segments with 4 control points
        #Get index of the segment
        nb_curve = (len(self.pointsL)-1)//3
        a = 1/nb_curve
        b = phase // a + 1
        indice_1 = int(b*3 - 3)

        #Get time inside the segment, t € [0,1]
        t_ref_poly = (phase % a )*nb_curve	

        points_control = [self.pointsL[indice_1 ], self.pointsL[indice_1 + 1] ,self.pointsL[indice_1 + 2] , self.pointsL[indice_1+ 3] ]

        pts_out = self.point_bezier_3(points_control, t_ref_poly)
        pts_out[0, 0] *= self.ratio[0]
        pts_out[0, 1] *= self.ratio[1]
        return pts_out
        
    
    # def plot(self):
    #     T = np.linspace(0,1,50)
    #     for elt in T:    
    #         [x,y] = self.getPos(elt)  
    #         plt.plot(x,y,'x')
	
def adapt(x):
    if(x < 0.75):
        return x*0.25/0.75
    return (-256/3)*x**3 + 224*x**2 - 575/3 * x + 54

def adaptMieux(x):
    if(x < 0.8):
        return x*0.25/0.8
    return 103125/8*x**5 - 928125/16*x**4 + 831875/8*x**3 - 185625/2*x**2 + 660005/16 * x - 7304

if __name__ == "__main__":
    import matplotlib.pyplot as plt
    points = [[-0.3625, 0.0, 0.3680, 0.0, 0.2019, 0.4846, -0.4183, 0.5634], [0.1733, 0.0, 0.176, 0.0, -0.2018, 0.1855, -0.2008, -0.1800]]
    footTraj1 = footTrajectoryBezier(points)


    # T = np.linspace(0,0.25,10)
    # for elt in T:    
    #     p = footTraj1.getPos(elt)
    #     plt.plot(p[0,0], p[0,1],'rx')
    
    # T = np.linspace(0.25,0.5,10)
    # for elt in T:    
    #     p = footTraj1.getPos(elt)
    #     plt.plot(p[0,0], p[0,1],'bx')

    # T = np.linspace(0.5,0.75,10)
    # for elt in T:    
    #     p = footTraj1.getPos(elt)
    #     plt.plot(p[0,0], p[0,1],'gx')

    # T = np.linspace(0.75,1,10)
    # for elt in T:    
    #     p = footTraj1.getPos(elt)
    #     plt.plot(p[0,0], p[0,1],'yx')

    T = np.linspace(0,1,100)
    for x in T:    
        plt.plot(x, adaptMieux(x),'bx')

    plt.show(block=False)
    plt.pause(0.001)
    while(True):
        pass
