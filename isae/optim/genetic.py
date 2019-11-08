from random import random, randrange
from sys import argv

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
			if(p < 0.35):
				child[g] = mom[g]
			elif(p < 0.70):
				child[g] = dad[g]
		newPop.append([0, child])

	return newPop

def print_pop(population):
	for people in population:
		temp_str = "\033[33m\t" + str(people[0]) + "\033[32m ==> \033[39m"
		for value in people[1]:
			temp_str += " " + str(value)
		print(temp_str)

def geneticSearch(popSize = 15, n_generation = 10):
	currGen = randomTries(n_tries = popSize)
	print("1st Generation : ")
	print_pop(currGen)

	for j in range(1, n_generation):
		currGen = getBest(popSize//3, currGen)
		print("Survivors (generation " + str(j+1) + ") : ")
		print_pop(currGen)
		currGen = mix(popSize, currGen)
		for k in range(popSize):
			currGen[k][0] = runSimu(currGen[k][1])
		print("New generation " + str(j+1) + " : ")
		print_pop(currGen)


if __name__ == "__main__":
    if(len(argv) != 3):
    	print("# Arguments must be :\n# # populationSize n_generation")
    	quit()
    geneticSearch(int(argv[1]), int(argv[2]))