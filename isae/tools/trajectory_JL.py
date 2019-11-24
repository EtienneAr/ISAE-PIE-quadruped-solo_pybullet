import numpy as np
import matplotlib.pyplot as plt
from math import pi,sqrt,cos,sin

def interpol(a, b, phase):
    return a + phase*(b-a)

class roundishTriangle:

    def __init__(self, length, height, summit_x_pos, radius):
        self.l_sol = length  #1
        self.r_turn = radius #0.1
        self.summit_x_pos = length*(summit_x_pos-0.5) #0 # different from 0.5 and -0.5
        self.height = height

        self.slope = (self.height-self.r_turn)/(summit_x_pos-(-self.l_sol/2))
        self.theta_lim3 = np.arctan2(self.height-self.r_turn,summit_x_pos-self.l_sol/2)-pi/2
        print(self.theta_lim3)


        self.r_turn1 = self.r_turn
        [xtemp, ytemp] = self.getPos_sol(1)
        self.x_c1 = xtemp
        self.y_c1 = ytemp + self.r_turn1
        
        self.theta_lim1 = np.arctan2(self.height - self.r_turn, summit_x_pos+self.l_sol/2)-2*pi+pi/2

        self.r_turn2 = self.r_turn
        self.x_c2 = summit_x_pos
        self.y_c2 = self.height

        self.A1 = self.getPos_turn1(1)
        self.B1 = self.getPos_turn2(0)
        self.a_monte = (self.B1[1]-self.A1[1])/(self.B1[0]-self.A1[0])
        self.b_monte = self.A1[1]-self.a_monte*self.A1[0]

        self.r_turn3 = self.r_turn
        [xtemp, ytemp] = self.getPos_sol(0)
        self.x_c3 = xtemp
        self.y_c3 = ytemp + self.r_turn3
        
        self.A2 = self.getPos_turn2(1)
        self.B2 = self.getPos_turn3(0)
        self.a_desc = (self.B2[1]-self.A2[1])/(self.B2[0]-self.A2[0])
        self.b_desc = self.A2[1]-self.a_desc*self.A2[0]

    def getPos_sol(self, subphase):
        x_alpha = interpol(self.l_sol/2, -self.l_sol/2, subphase)
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

    def getPos(self, phase, factor = None):
        if phase < 0.3:
            prctg = phase/0.3
            return self.getPos_sol(prctg)

        if phase < 0.4:
            prctg = (phase-0.3)/0.1
            return self.getPos_turn1(prctg)

        if phase < 0.6:
            prctg = (phase-0.4)/0.2
            return self.getPos_monte(prctg)

        if phase < 0.7:
            prctg = (phase-0.6)/0.1
            return self.getPos_turn2(prctg)

        if phase < 0.9:
            prctg = (phase-0.7)/0.2
            return self.getPos_desc(prctg)

        if phase <= 1:
            prctg = (phase-0.9)/0.1
            return self.getPos_turn3(prctg)
        
        return None


rd = roundishTriangle(1,1,0.37,0.1)
for i in range(100):
    [x,y] = rd.getPos(i/100.)
    plt.plot(x,y,"b+")


plt.title("Trajectoire pied")
plt.legend()
plt.xlabel("Position relative du pied")
plt.ylabel("Hauteur relative du pied par rapport au point d'ancrage")

plt.show() # affiche la figure a l'ecran
