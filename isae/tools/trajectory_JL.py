import numpy as np
from math import pi,sqrt,cos,sin

def interpol(a, b, phase):
    return a + phase*(b-a)

class roundishTriangle:

    def __init__(self, length, height, radius, center):
        self.l_gnd = length  #1
        self.r_turn = radius #0.1
        self.height = height # 1
        self.center = center # 0

        #Setl variables necessary for computation

        self.slope = (self.height-self.r_turn)/(0-(-self.l_gnd/2))
        self.theta_lim3 = np.arctan2(self.height-self.r_turn,-self.l_gnd/2)-pi/2

        self.r_turn1 = self.r_turn
        [xtemp, ytemp] = self.getPos_gnd(1)
        self.x_c1 = xtemp
        self.y_c1 = ytemp + self.r_turn1
        
        self.theta_lim1 = np.arctan2(self.height - self.r_turn, self.l_gnd/2)-2*pi+pi/2

        self.r_turn2 = self.r_turn
        self.x_c2 = center
        self.y_c2 = self.height

        self.A1 = self.getPos_turn1(1)
        self.B1 = self.getPos_turn2(0)
        self.a_monte = (self.B1[1]-self.A1[1])/(self.B1[0]-self.A1[0])
        self.b_monte = self.A1[1]-self.a_monte*self.A1[0]

        self.r_turn3 = self.r_turn
        [xtemp, ytemp] = self.getPos_gnd(0)
        self.x_c3 = xtemp
        self.y_c3 = ytemp + self.r_turn3
        
        self.A2 = self.getPos_turn2(1)
        self.B2 = self.getPos_turn3(0)
        self.a_desc = (self.B2[1]-self.A2[1])/(self.B2[0]-self.A2[0])
        self.b_desc = self.A2[1]-self.a_desc*self.A2[0]

        #Normalize speed
        l_t1 = abs(self.theta_lim1 + pi/2) * self.r_turn1
        l_monte = sqrt((self.A1[0]-self.B1[0])**2 + (self.A1[1]-self.B1[1])**2)
        l_t2 = abs(self.theta_lim3-2*pi - self.theta_lim1) * self.r_turn2
        l_desc = sqrt((self.A2[0]-self.B2[0])**2 + (self.A2[1]-self.B2[1])**2)
        l_t3 = abs(-pi/2 - self.theta_lim3) * self.r_turn3

        self.phases = [0 for i in range(6)]
        
        self.phases[0] = 0.5 #ground - half the motion
        self.phases[1] = 0.5 * l_t1 / self.l_gnd #turn 1 - same speed as ground
        self.phases[-1] = 0.5 * l_t3 / self.l_gnd #turn 3 - same speed as ground
        
        phase_rest = 1.0 - sum(self.phases)
        l_rest = l_monte + l_t2 + l_desc

        self.phases[1] = phase_rest * l_monte / l_rest #monte - same speed as turn 2 and desc
        self.phases[1] = phase_rest * l_t2    / l_rest #turn 2 - same speed as monte and desc
        self.phases[1] = phase_rest * l_desc  / l_rest #desc - same speed as monte and turn2
        
        print(self.phases)
        


    def getPos_gnd(self, subphase):
        x_alpha = interpol(self.l_gnd/2, -self.l_gnd/2, subphase)
        y_alpha = 0
        return [x_alpha, y_alpha]

    def getPos_turn1(self, subphase):
        theta_turn1 = interpol(-pi/2, self.theta_lim1, subphase)
        x_turn1 = self.x_c1 + self.r_turn1*cos(theta_turn1)
        y_turn1 = self.y_c1 + self.r_turn1*sin(theta_turn1)
        return [x_turn1, y_turn1]

    def getPos_turn2(self, subphase):
        theta_turn2 = interpol(self.theta_lim1, self.theta_lim3-2*pi,subphase)
        x_turn2 = self.x_c2+ self.r_turn2*cos(theta_turn2)
        y_turn2 = self.y_c2+ self.r_turn2*sin(theta_turn2)
        return [x_turn2, y_turn2]

    def getPos_monte(self, subphase):
        x_monte = interpol(self.A1[0], self.B1[0], subphase)
        y_monte = self.a_monte*x_monte+self.b_monte
        return [x_monte, y_monte]

    def getPos_turn3(self, subphase):
        theta_turn3 = interpol(self.theta_lim3, -pi/2, subphase)
        x_turn3 = self.x_c3 + self.r_turn3*cos(theta_turn3)
        y_turn3 = self.y_c3 + self.r_turn3*sin(theta_turn3)
        return [x_turn3, y_turn3]

    def getPos_desc(self, subphase):
        x_desc = interpol(self.A2[0], self.B2[0], subphase)
        y_desc = self.a_desc*x_desc+self.b_desc
        return [x_desc, y_desc]

    def getPos_ref(self, phase, factor = None):
        #i determines to which part of the motion this phase correspond
        #then subphase is the progression in this particular part of the mouvement. 
        i=0
        subphase = phase%1
        while(subphase > self.phases[i]):
            subphase -= self.phases[i]
            i += 1
        subphase /= self.phases[i]

        if(i==0):
            return self.getPos_gnd(subphase)

        if(i==1):
            return self.getPos_turn1(subphase)

        if(i==2):
            return self.getPos_monte(subphase)

        if(i==3):
            return self.getPos_turn2(subphase)

        if(i==4):
            return self.getPos_desc(subphase)

        if(i==5):
            return self.getPos_turn3(subphase)
        
        return None

    def getPos(self, phase, factor = None):
        [x, y] = self.getPos_ref(phase, factor)
        return [x, y]


# # Debug 
# import matplotlib.pyplot as plt
# rd = roundishTriangle(1.1,0.5,0.05,0,)
# for i in range(100):
#     [x,y] = rd.getPos(i/100.)
#     plt.plot(x,y,"b+")


# plt.title("Trajectoire pied")
# plt.legend()
# plt.xlabel("Position relative du pied")
# plt.ylabel("Hauteur relative du pied par rapport au point d'ancrage")

# plt.show() # affiche la figure a l'ecran
