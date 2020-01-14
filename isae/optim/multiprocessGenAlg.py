from isae.optim.genAlg import *
from isae.sim_control.gradedSimulation import *
import random
import string

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
        BLUE = "\033[34m"
        GREEN = "\033[32m"
        DEFAULT = "\033[39m"
        YELLOW = "\033[33m"

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