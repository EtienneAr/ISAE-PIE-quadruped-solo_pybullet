from random import random, randrange

from isae.optim.evaluate import parametersRange, runSimu, randomConfig, randomTries


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
