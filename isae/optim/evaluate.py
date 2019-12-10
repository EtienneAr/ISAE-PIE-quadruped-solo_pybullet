import subprocess
from random import random

parametersRange = [ [0,2], #bodyHeight
					[0,4], #stepPeriod
					[-2,2], #stepLen
					[0,1], [0,1], [0,1], [0,1], #phasesOff
					[-2,2], [0,2], #point0 X,Y
					[-2,2], [0,2], #point1 X,Y
					[-2,2], [0,2], #point2 X,Y
					[0,40], #Kp
					[0,30], #Kd
					]

def runSimu(args):
	try:
		ret = subprocess.check_output(["python3 -m solo_pybullet False False " + ' '.join(map(str, args))], shell=True, universal_newlines=True)
		lines = ret.split('\n')
		result = -1
		for i in range(len(lines)):
			if(lines[i] == "Result :"):
				result = i+1
				break

		if result > -1:
			return float(lines[result])
	except KeyboardInterrupt:
		raise KeyboardInterrupt
	except:
		pass
	return -1000.


def randomConfig():
	return list(map(lambda bounds: bounds[0] + random() * (bounds[1] - bounds[0]), parametersRange))

def randomTries(n_tries = 10):
	result = []

	for _ in range(n_tries):
		params = randomConfig()
		result.append([runSimu(params), params])

	return result