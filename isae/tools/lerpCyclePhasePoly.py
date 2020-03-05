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
        # polynome de degre 2
        x=(1-t)**2
        y=t*t
        A = self.combinaison_lineaire(self.P0,self.P1,(1-t)*x,3*t*x)
        B = self.combinaison_lineaire(self.P2,self.P3,3*y*(1-t),y*t)
        return [A[0]+B[0],A[1]+B[1]]

    def lerpCyclePhase_3(self,t):
		#polynomial of degree 3
        self.x0 = 0
        return self.point_bezier_3(t)[0]

    def lerpCyclePhase_bezier(self,t):
        #Bezier curve
        if t == 1:
            return 1
        if t == 0:
            return 0
        else:
            self.x0 = t
            a = 3*self.P1[0] - 3*self.P2[0] + 1
            b = -6*self.P1[0] + 3*self.P2[0]
            c = 3*self.P1[0]
            d = - self.x0
            L = solve(a,b,c,d)      
            x = 0        
            for elt in L :
                if (elt.real <= 1.) & (elt.real >= 0) & (abs(elt.imag) <= 10e-4) :
                    x = elt.real

            return self.point_bezier_3(x)[1]

    


# Main Function takes in the coefficient of the Cubic Polynomial
# as parameters and it returns the roots in form of numpy array.
# Polynomial Structure -> ax^3 + bx^2 + cx + d = 0

def solve(a, b, c, d):

    if (a == 0 and b == 0):                     # Case for handling Liner Equation
        return np.array([(-d * 1.0) / c])                 # Returning linear root as numpy array.

    elif (a == 0):                              # Case for handling Quadratic Equations

        D = c * c - 4.0 * b * d                       # Helper Temporary Variable
        if D >= 0:
            D = math.sqrt(D)
            x1 = (-c + D) / (2.0 * b)
            x2 = (-c - D) / (2.0 * b)
        else:
            D = math.sqrt(-D)
            x1 = (-c + D * 1j) / (2.0 * b)
            x2 = (-c - D * 1j) / (2.0 * b)
            
        return np.array([x1, x2])               # Returning Quadratic Roots as numpy array.

    f = findF(a, b, c)                          # Helper Temporary Variable
    g = findG(a, b, c, d)                       # Helper Temporary Variable
    h = findH(g, f)                             # Helper Temporary Variable

    if f == 0 and g == 0 and h == 0:            # All 3 Roots are Real and Equal
        if (d / a) >= 0:
            x = (d / (1.0 * a)) ** (1 / 3.0) * -1
        else:
            x = (-d / (1.0 * a)) ** (1 / 3.0)
        return np.array([x, x, x])              # Returning Equal Roots as numpy array.

    elif h <= 0:                                # All 3 roots are Real

        i = math.sqrt(((g ** 2.0) / 4.0) - h)   # Helper Temporary Variable
        j = i ** (1 / 3.0)                      # Helper Temporary Variable
        k = math.acos(-(g / (2 * i)))           # Helper Temporary Variable
        L = j * -1                              # Helper Temporary Variable
        M = math.cos(k / 3.0)                   # Helper Temporary Variable
        N = math.sqrt(3) * math.sin(k / 3.0)    # Helper Temporary Variable
        P = (b / (3.0 * a)) * -1                # Helper Temporary Variable

        x1 = 2 * j * math.cos(k / 3.0) - (b / (3.0 * a))
        x2 = L * (M + N) + P
        x3 = L * (M - N) + P

        return np.array([x1, x2, x3])           # Returning Real Roots as numpy array.

    elif h > 0:                                 # One Real Root and two Complex Roots
        R = -(g / 2.0) + math.sqrt(h)           # Helper Temporary Variable
        if R >= 0:
            S = R ** (1 / 3.0)                  # Helper Temporary Variable
        else:
            S = (-R) ** (1 / 3.0) * -1          # Helper Temporary Variable
        T = -(g / 2.0) - math.sqrt(h)
        if T >= 0:
            U = (T ** (1 / 3.0))                # Helper Temporary Variable
        else:
            U = ((-T) ** (1 / 3.0)) * -1        # Helper Temporary Variable

        x1 = (S + U) - (b / (3.0 * a))
        x2 = -(S + U) / 2 - (b / (3.0 * a)) + (S - U) * math.sqrt(3) * 0.5j
        x3 = -(S + U) / 2 - (b / (3.0 * a)) - (S - U) * math.sqrt(3) * 0.5j

        return np.array([x1, x2, x3])           # Returning One Real Root and two Complex Roots as numpy array.

# Helper function to return float value of f.
def findF(a, b, c):
    return ((3.0 * c / a) - ((b ** 2.0) / (a ** 2.0))) / 3.0


# Helper function to return float value of g.
def findG(a, b, c, d):
    return (((2.0 * (b ** 3.0)) / (a ** 3.0)) - ((9.0 * b * c) / (a **2.0)) + (27.0 * d / a)) /27.0


# Helper function to return float value of h.
def findH(g, f):
    return ((g ** 2.0) / 4.0 + (f ** 3.0) / 27.0)