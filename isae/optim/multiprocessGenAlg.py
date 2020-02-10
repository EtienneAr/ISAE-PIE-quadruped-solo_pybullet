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
        self.CATCH_ERROR = True

    def __call__(self, pop_indiv):
        BLUE = Fore.BLUE
        GREEN = Fore.GREEN
        DEFAULT = Fore.RESET
        YELLOW = Fore.YELLOW

        sim = self.simFromParam(pop_indiv)
        sim.initializeSim()
        grade = -10e6
        try:
            sim.runSim()
            grade = sim.grades[self.grade_index]
        except Exception as e:
            if not self.CATCH_ERROR:
                raise e

        print(BLUE + "Indiv. " + randomString() + " : " + DEFAULT)
        print(self.getIndivArray(pop_indiv))
        print(GREEN + "Fitness : " + str(grade) + DEFAULT)
        print('\n')

        return [grade, pop_indiv, self.getIndivArray(pop_indiv)]

    def gradePopulation(self, pop):        
        with mp.Pool(mp.cpu_count()) as pool:
            gradedPop = pool.map(self, pop)
        return gradedPop