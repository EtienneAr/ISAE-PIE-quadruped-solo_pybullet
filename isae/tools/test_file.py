from new_trajectory import *

contTraj = continuousTrajectory([[-1,0],[-0.5,1],[0.5,1],[1,0], [-1,0]])
sampledTraj = contTraj.toSampledTraj([i/20.0 for i in range(21)])

contTraj.plot()

sampledTraj.plot()
plt.show()