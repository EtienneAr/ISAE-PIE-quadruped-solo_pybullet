import matplotlib.pyplot as plt

def debug_footTrajectory(traj, pointsNb):
	t = [1.*i/(pointsNb-1) for i in range(pointsNb)]
	xy = list(map(traj.getPos, t))
	x = list(map(lambda c: c[0], xy))
	y = list(map(lambda c: c[1], xy))
	plt.plot(x, y, '-o')
	plt.show()
	#input("Press enter to close")


from trajectory import *
# t = pointsTrajectory(10, [[-5,1],[-1,2],[5,0.5]], 25)
t = pointsTrajectory([[-1,0.5], [-0.5,1], [1,0.5]], factor = [10,10])
debug_footTrajectory(t, 50)