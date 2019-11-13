from new_trajectory import *
from scipy.spatial import ConvexHull

def polynomIn0_1(coeffs, sampleN):
    coeffs = coeffs / np.sum(coeffs)
    values = []
    for i in range(int(sampleN + 1)):
        s = 0
        for k in range(len(coeffs)):
            s += coeffs[k]*(i/N)**(k+1)
        values.append(s)
    return np.array(values)

def invPolynomIn0_1(coeffs, sampleN):
    coeffs = coeffs / np.sum(coeffs)
    values = []
    for i in range(int(sampleN + 1)):
        s = 0
        for k in range(len(coeffs)):
            s += coeffs[k]*(i/N)**(1.0/(k+1))
        values.append(s)
    return np.array(values)

xfactor = 0.5 + np.random.random()
yfactor = 0.5 + np.random.random()
randomPoints = np.random.random((10,2))

chull = ConvexHull(randomPoints)
shape = randomPoints[chull.vertices]
shape = np.concatenate((shape,[shape[0]]))
shape[:,0] *= xfactor
shape[:,1] *= yfactor

N = 100.0
vp_coeffs = [0,-1,1.5,1.5,0,0,0] # sum to 1
inv_vp_coeffs = [0,1,1.5,1.5,0,0,0] # sum to 1

random1 = 0.25 + np.random.random(10)
random2 = -0.5 + np.random.random(5)

linear = [1]

vel_profile = 0.5*polynomIn0_1(linear , N) + 0.5*invPolynomIn0_1(linear , N)      

contTraj = continuousTrajectory([[-1,0],[0,2],[1,0], [-1,0]])
#contTraj = continuousTrajectory(shape)
sampledTraj = contTraj.toSampledTraj(vel_profile)

contPoints = np.array(sampledTraj.points)
segLengths = (np.diff(contPoints[:,0])**2 + np.diff(contPoints[:,1])**2)**.5 # segment lengths, size n
cumulLength = np.concatenate(([0],np.cumsum(segLengths))) # cumulated lengths, size n

contTraj.plot()

sampledTraj.plot()

print(sampledTraj.newGetPos(0.5,cumulLength))
#plt.scatter([sampledTraj.getPos(0.05*i)[0] for i in range(21)],[sampledTraj.getPos(0.05*i)[1] for i in range(21)])

plt.figure()
plt.plot(vel_profile,'-o')
plt.title("Vel profile")
plt.show()