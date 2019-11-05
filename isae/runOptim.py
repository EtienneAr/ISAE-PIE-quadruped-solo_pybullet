import subprocess
from random import random, randrange

parametersRange = [[0,2], #bodyHeight
					[-1,1], #pointTraj_x
					[0,1], #pointTraj_y
					[0,1], [0,1], [0,1], [0,1]] #phasesOff

def runSimu(args):
	ret = subprocess.check_output(["python3 -m solo_pybullet False False " + ' '.join(map(str, args))], shell=True, universal_newlines=True)

	lines = ret.split('\n')
	result = -1
	for i in range(len(lines)):
		if(lines[i] == "Result :"):
			result = i+1
			break

	if result > -1:
		return float(lines[result])

	return None


def randomConfig():
	return list(map(lambda bounds: bounds[0] + random() * (bounds[1] - bounds[0]), parametersRange))

def randomTries(n_tries = 10):
	result = []

	for _ in range(n_tries):
		params = randomConfig()
		result.append([runSimu(params), params])

	return result


def getBest(toKeep, population):
	population.sort(key=lambda p: -p[0])
	return population[:toKeep]

def mix(newSize, population):
	newPop = population[:]

	for _ in range(newSize - len(population)):
		mom = population[randrange(len(population))][1]
		dad = population[randrange(len(population))][1]
		child = randomConfig()
		for g in range(len(child)):
			p = random()
			if(p < 0.4):
				child[g] = mom[g]
			elif(p < 0.8):
				child[g] = dad[g]
		newPop.append([0, child])

	return newPop



def geneticSearch(popSize = 15, n_generation = 10):
	currGen = randomTries(n_tries = popSize)
	print("1st Gen : ")
	print(currGen)

	for j in range(1, n_generation):
		currGen = getBest(popSize//3, currGen)
		print("Survivors (generation " + str(j) + ") : ")
		print(currGen)
		currGen = mix(popSize, currGen)
		for k in range(popSize):
			currGen[k][0] = runSimu(currGen[k][1])
		print("New generation " + str(j) + " : ")
		print(currGen)
