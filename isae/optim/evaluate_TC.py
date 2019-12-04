import subprocess
from random import random
import numpy as np

parametersRange = [[0.2,1.8], #bodyHeight
					[2,4], #stepPeriod
					[-2,2], #stepLen
					[0,1], [0,1], [0,1], [0,1], #phasesOff
					[0,40], #Kp
					[0,30], #Kd
					]


def runSimu(args):
	try:
		ret = subprocess.check_output(["python3 -m solo_pybullet False False 13 " + ' '.join(map(str, args))], shell=True, universal_newlines=True)
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
		return -1000.


def randomConfig():
	

	L = []

	for k in range(7):
		bounds = parametersRange[k]
		L.append(bounds[0] + random() * (bounds[1] - bounds[0]))


	P0 = [-0.25,0.25]
	P3 = [0.25 , 0.25]
	P6 = [0.3,0.0]
	P9 = [-0.3 , 0.0]
	

	L.append(P0[0])
	L.append(P0[1])
	P1 = P0 + 0.55*np.random.rand(2)
	P2 = P0 + 0.55*np.random.rand(2)
	L.append(P1[0])
	L.append(P1[1])
	L.append(P2[0])
	L.append(P2[1])
	L.append(P3[0])
	L.append(P3[1])

	x = L[-2] + (L[-2] - L[-4])
	y = L[-1] + (L[-1] - L[-3])
	P4 = [x , y]
	L.append(P4[0])
	L.append(P4[1])

	P5 = np.zeros(2)
	P5[0] = P3[0] + 0.8*np.random.rand(1)
	P5[1] = 0.0
	L.append(P5[0])
	L.append(P5[1])
	L.append(P6[0])
	L.append(P6[1])

	x = L[-2] + (L[-2] - L[-4])
	y = L[-1] + (L[-1] - L[-3])
	P7 = [x , y]
	L.append(P7[0])
	L.append(P7[1])

	P8 = np.zeros(2)
	P8[0] = P6[0] - 0.35*np.random.rand(1)
	P8[1] = 0.0
	L.append(P8[0])
	L.append(P8[1])
	L.append(P9[0])
	L.append(P9[1])

	x = L[-2] + (L[-2] - L[-4])
	y = L[-1] + (L[-1] - L[-3])
	P10 = [x , y]
	L.append(P10[0])
	L.append(P10[1])

	x = L[-2] + (L[-2] - L[-4])
	y = L[-1] + (L[-1] - L[-3])
	P11 = [x , y]

	L.append(P11[0])
	L.append(P11[1])
	P12 = P0
	L.append(P12[0])
	L.append(P12[1])
	

	for k in range(-2,0,1):
		bounds = parametersRange[k]
		L.append(bounds[0] + random() * (bounds[1] - bounds[0]))

	return L 


def randomTries(n_tries = 10):
	result = []

	for _ in range(n_tries):
		params = randomConfig()
		result.append([runSimu(params), params])

	return result





