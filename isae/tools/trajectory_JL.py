import numpy as np
import matplotlib.pyplot as plt
from math import pi,sqrt,cos,sin

l_sol = 1
r_turn = 0.1

height = 0.6
slope = (height-r_turn)/(0-(-l_sol/2))
theta_lim3 = np.arctan2(height-r_turn,-l_sol/2)-pi/2
n_straight = 20
x_sol = np.linspace(l_sol/2, -l_sol/2,n_straight)
y_sol = [0 for i in range(len(x_sol))]

r_turn1 = r_turn
x_c1 = x_sol[n_straight-1]
y_c1 = y_sol[n_straight-1]+r_turn1
n_turn = 50
theta_lim1 = np.arctan2(height-r_turn,l_sol/2)-2*pi+pi/2
theta_turn1 = np.linspace(-pi/2,theta_lim1,n_turn)
x_turn1 = x_c1+ r_turn1*np.cos(theta_turn1)
y_turn1 = y_c1+ r_turn1*np.sin(theta_turn1)

r_turn2 = r_turn
x_c2 = 0
y_c2 = height
theta_turn2 = np.linspace(theta_lim1,theta_lim3-2*pi,n_turn/2)
x_turn2 = x_c2+ r_turn2*np.cos(theta_turn2)
y_turn2 = y_c2+ r_turn2*np.sin(theta_turn2)

A1 = [x_turn1[n_turn-1],y_turn1[n_turn-1]]
B1 = [x_turn2[0],y_turn2[0]]
a_monte = (B1[1]-A1[1])/(B1[0]-A1[0])
b_monte = A1[1]-a_monte*A1[0]
x_monte = np.linspace(x_turn1[n_turn-1],x_turn2[0],n_straight)
y_monte = a_monte*x_monte+b_monte

r_turn3 = r_turn
x_c3 = x_sol[0]
y_c3 = y_sol[0]+r_turn3
theta_turn3 = np.linspace(theta_lim3,-pi/2,n_turn)
x_turn3 = x_c3+ r_turn3*np.cos(theta_turn3)
y_turn3 = y_c3+ r_turn3*np.sin(theta_turn3)

A2 = [x_turn2[int(n_turn/2-1)],y_turn2[int(n_turn/2-1)]]
B2 = [x_turn3[0],y_turn3[0]]
a_desc = (B2[1]-A2[1])/(B2[0]-A2[0])
b_desc = A2[1]-a_desc*A2[0]
x_desc = np.linspace(x_turn2[int(n_turn/2-1)],x_turn3[0],n_straight)
y_desc = a_desc*x_desc+b_desc

def alpha(x):
    if x <=0.3:
        prctg = x/0.3
        x_alpha = x_sol[int(n_straight*prctg)]
        y_alpha = 0
    else :
        if x <=0.4:
            prctg = (x-0.3)/0.1
            x_alpha = x_turn1[int(n_turn*prctg)]
            y_alpha = y_turn1[int(n_turn*prctg)]
        else :
            if x <=0.6:
                prctg = (x-0.4)/0.2
                x_alpha = x_monte[int(n_straight*prctg)]
                y_alpha = y_monte[int(n_straight*prctg)]
            else :
                if x <=0.7:
                    prctg = (x-0.6)/0.1
                    x_alpha = x_turn2[int(n_turn/2*prctg)]
                    y_alpha = y_turn2[int(n_turn/2*prctg)]
                else :
                    if x <=0.9:
                        prctg = (x-0.7)/0.2
                        x_alpha = x_desc[int(n_straight*prctg)]
                        y_alpha = y_desc[int(n_straight*prctg)]
                    else :
                        prctg = (x-0.9)/0.1
                        x_alpha = x_turn3[int(n_turn/2*prctg)]
                        y_alpha = y_turn3[int(n_turn/2*prctg)]
    return [x_alpha,y_alpha]

[x,y] = alpha(0.62)
print(x,y)

plt.plot(x_sol,y_sol,"b--",label="contact")
plt.plot(x_monte,y_monte,"g--",label="montée")
plt.plot(x_desc,y_desc,"g--",label="descente")
plt.plot(x_c1,y_c1,"ro")
plt.plot(x_turn1,y_turn1,"r--",label="courbe 1")
plt.plot(x_c2,y_c2,"ro")
plt.plot(x_turn2,y_turn2,"r--",label="courbe 2")
plt.plot(x_c3,y_c3,"ro")
plt.plot(x_turn3,y_turn3,"r--",label="courbe 3")
plt.plot(x,y,"bo",label="target")


plt.xlim(-l_sol, l_sol)
plt.ylim(-0.1,1)
plt.title("Trajectoire pied")
plt.legend()
plt.xlabel("Position relative du pied")
plt.ylabel("Hauteur relative du pied par rapport au point d'ancrage")

plt.show() # affiche la figure a l'ecran
