from random import random, randrange
from sys import argv

BLUE = "\033[34m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
DEFAULT = "\033[39m"

from isae.optim.evaluate_TC import parametersRange, runSimu, randomConfig, randomTries

file1 = open("MyFile.txt","a") 

def getBest(toKeep, population):
	for elt in population:
		print(elt[0])

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
		temp_str = YELLOW + str(people[0]) + GREEN + " ==> " + DEFAULT
		for value in people[1]:
			temp_str += " " + str(value)
		print(temp_str)

def geneticSearch(popSize = 15, n_generation = 10):
	currGen = randomTries(n_tries = popSize)
	print(BLUE + " #### \n1st Generation : " + DEFAULT)
	print_pop(currGen)

	for j in range(1, n_generation):
		currGen = getBest(popSize//3, currGen)
		temp_str = ''
		for value in currGen[0][1]:
			temp_str += " " + str(value)
		file1.write(temp_str)
		file1.write('\n')
		print(BLUE + "####")
		print("Survivors (generation " + str(j) + ") : ")
		print_pop(currGen)
		currGen = mix(popSize, currGen)
		for k in range(popSize):
			currGen[k][0] = runSimu(currGen[k][1])
		print(CYAN + "####")
		print("New generation " + str(j+1) + " : ")
		print_pop(currGen)


if __name__ == "__main__":
    if(len(argv) != 3):
    	print(YELLOW + "# Arguments must be : " + CYAN + "populationSize   n_generation" + DEFAULT)
    	quit()
    geneticSearch(int(argv[1]), int(argv[2]))