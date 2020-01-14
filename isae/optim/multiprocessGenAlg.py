from isae.optim.genAlg import *
from isae.sim_control.gradedSimulation import *
import random
import string
from colorama import Fore
import multiprocessing as mp

def randomString(stringLength=7):
    """Generate a random string of fixed length """
    letters = string.ascii_letters
    return ''.join(random.choice(letters) for i in range(stringLength))

# use multiprocessing to speed up the optimization

class multiprocessGeneticAlgorithm(geneticAlgorithm):
    def __init__(self):
        geneticAlgorithm.__init__(self)

    def __call__(self, pop_indiv):
        BLUE = Fore.BLUE
        GREEN = Fore.GREEN
        DEFAULT = Fore.RESET
        YELLOW = Fore.YELLOW

        sim = self.simFromParam(pop_indiv)
        sim.initializeSim()
        sim.runSim()
        print(BLUE + "Indiv. " + randomString() + " : " + DEFAULT)
        print(self.getIndivArray(pop_indiv))
        print(GREEN + "Fitness : " + str(sim.grades[self.grade_index]) + DEFAULT)
        print('\n')

        return [sim.grades[self.grade_index], pop_indiv, self.getIndivArray(pop_indiv)]

    def gradePopulation(self, pop):
        pool = mp.Pool(mp.cpu_count())
        gradedPop = pool.map(self, pop)

        return gradedPop