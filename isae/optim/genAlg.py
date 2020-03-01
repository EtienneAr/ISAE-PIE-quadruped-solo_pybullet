from isae.optim.genAlgParam import *
from isae.sim_control.gradedSimulation import *
from colorama import Fore, Back

class geneticAlgorithm(object):

    def __init__(self):
        self.paramTypes = [] # possible types : "scalar", "2dPoint"
        self.paramArgs = [] # corresponding args : [minVal, maxVal], [[xMin, xMax],[yMin, yMax]]
        self.paramNames = [] # ex : ["Period", "Kp"]
        self.pop_size = 4
        self.n_gen = 1
        self.grade_index = 0
        self.best_per_gen = []
        self.genLog = []
        self.paramToSim = None # expects a function paramToSim(paramsInstance) that returns a gradedSimulation with parameters set
                                # this function has to be defined from outside,
                                # which allows to use any simulation parameters as parameters for optimization

    def setParamTypes(self,paramTypes):
        self.paramTypes = paramTypes

    def setParamArgs(self,paramArgs):
        self.paramArgs = paramArgs

    def setParamNames(self,paramNames):
        self.paramNames = paramNames

    def printParamsInstance(self, paramsInstance):
        for k in range(len(paramsInstance)):
            print(str(paramsInstance[k].value) + ", \t#" + Fore.BLACK + Back.WHITE + self.paramNames[k] + Fore.RESET + Back.RESET)

    def getIndivArray(self, indiv):
        return np.array([p.toArray() for p in indiv])

    def setParamToSim(self, paramToSim):
        self.paramToSim = paramToSim

    def simFromParam(self, paramsInstance):
        return self.paramToSim(paramsInstance)

    # initializes a random population, using GA_<param>.initRandom(args) with args specified in self.paramArgs
    def initRandomPopulation(self):
        pop = []
        for k in range(self.pop_size):
            indiv = []
            for i in range(len(self.paramTypes)):
                if(self.paramTypes[i] == "scalar"):
                    paramInstance = GA_scalar(self.paramArgs[i])
                if(self.paramTypes[i] == "scalarBinary"):
                    paramInstance = GA_scalarBinary(self.paramArgs[i])
                if(self.paramTypes[i] == "2dPoint"):
                    paramInstance = GA_2dPoint(self.paramArgs[i])
                if(self.paramTypes[i] == "ptFtTraj"):
                    paramInstance = GA_pointFootTraj(self.paramArgs[i])
                if(self.paramTypes[i] == "legsOffsets"):
                    paramInstance = GA_legsOffsets(self.paramArgs[i])
                paramInstance.initRandom()
                indiv.append(paramInstance)
            pop.append(indiv)
        return pop
    
    # initializes a population from a list of given solutions
    # the population is populated with self.pop_size/n instances of each individual in the list, n being the length of this list
    # indivList is a list of GA_<param>.value, as defined in the respective GA_<param> classes
    def initFromIndiv(self, indivList):
        pop = []
        n = len(indivList)
        for k in range(n):
            for j in range(int(self.pop_size/n)):
                indiv = []
                for i in range(len(self.paramTypes)):
                    if(self.paramTypes[i] == "scalar"):
                        paramInstance = GA_scalar(self.paramArgs[i], value=indivList[k][i])
                    if(self.paramTypes[i] == "2dPoint"):
                        paramInstance = GA_2dPoint(self.paramArgs[i], value=indivList[k][i])
                    if(self.paramTypes[i] == "ptFtTraj"):
                        paramInstance = GA_pointFootTraj(self.paramArgs[i], value=indivList[k][i])
                    if(self.paramTypes[i] == "legsOffsets"):
                        paramInstance = GA_legsOffsets(self.paramArgs[i], value=indivList[k][i])
                    indiv.append(paramInstance)
                pop.append(indiv)
        return pop
    
    def initFromPopLog(self, popLog):
        pop = []
        n = len(popLog[1])
        for k in range(n):
            indiv = []
            params = popLog[1,k]
            for i in range(len(self.paramTypes)):
                if(self.paramTypes[i] == "scalar"):
                    paramInstance = GA_scalar(self.paramArgs[i], value=params[i])
                if(self.paramTypes[i] == "2dPoint"):
                    paramInstance = GA_2dPoint(self.paramArgs[i], value=params[i])
                if(self.paramTypes[i] == "ptFtTraj"):
                    paramInstance = GA_pointFootTraj(self.paramArgs[i], value=params[i])
                if(self.paramTypes[i] == "legsOffsets"):
                    paramInstance = GA_legsOffsets(self.paramArgs[i], value=params[i])
                indiv.append(paramInstance)
            pop.append(indiv)
        #TO DO : complete with random indiv
        return pop

    # initializes the simulations defined by the given pop and runs them while computing the optimization metrics
    # returns an array that is consumed by the folowing steps of the optimization process,
    # and appends the current population to the optimization log
    def gradePopulation(self, pop):
        BLUE = Fore.BLUE
        GREEN = Fore.GREEN
        DEFAULT = Fore.RESET
        YELLOW = Fore.YELLOW

        gradedPop = []
        popLog = []
        for i in range(len(pop)):
            indiv = pop[i]
            sim = self.simFromParam(indiv)
            sim.initializeSim()
            sim.runSim()
            # Choice of optimization function in sim.grades list
            gradedPop.append([sim.grades[self.grade_index], indiv, self.getIndivArray(indiv)])
            print(BLUE + "Indiv. " + str(i) + " : " + DEFAULT)
            print(self.getIndivArray(indiv))
            print(GREEN + "Fitness : " + str(sim.grades[self.grade_index]) + DEFAULT)
            print('\n')
        self.genLog.append(popLog)
        return gradedPop
    
    # sorts the population returned by self.gradePopulation() on descending order of score (fitness)
    def sortGradedPopulation(self, gradedPop):
        gradedPop.sort(key=lambda p: -p[0])
        popParams = np.array(gradedPop)[:,2]
        popFitness = np.array(gradedPop)[:,0]
        self.genLog.append([popFitness, popParams])

    # selects the part of the population that will go on with the optimization
    # and be used to recreate new solutions
    def selectBest(self, gradedPop, propToKeep = 0.5):
        n = int(propToKeep*len(gradedPop))
        return gradedPop[:n]

    def reprodPopulation(self, gradedPop, newSize):
        rand.shuffle(gradedPop)
        newPop = []

        for p in gradedPop:
            newPop.append(p[1])

        for k in range(int((newSize - len(gradedPop))/2)):
            p1 = int(rand.random() * len(gradedPop))
            p2 = int(rand.random() * len(gradedPop))
            parent1 = gradedPop[p1][1]
            parent2 = gradedPop[p2][1]
            child1 = []
            child2 = []
            for i in range(len(parent1)):
                cp1, cp2 = parent1[i].mergeWith(parent2[i])
                child1.append(cp1)
                child2.append(cp2)
            newPop.append(child1)
            newPop.append(child2)

        return newPop
    
    # applies a mutation to the individuals with a propbability defined for each of their GA_<param>
    def mutatePopulation(self,gradedPop):
        for indiv in gradedPop:
            for gene in indiv:
                gene.mutate()

    # puts it all together to run the full optimization process
    def runOptim(self, fromIndiv=None, fromPop=None):
        
        init_time = time.time()
        CYAN = Fore.CYAN
        RED = Fore.RED
        DEFAULT = Fore.RESET

        if fromPop == None:
            pop = self.initRandomPopulation()
        #elif fromPop == None:
        #    pop = self.initFromIndiv(fromIndiv)
        else:
            pop = self.initFromPopLog(fromPop)
        for k in range(self.n_gen):
            print("\n\n")
            print(CYAN + "GEN " + str(k) + DEFAULT)
            print("Pop size : {}".format(len(pop)))
            pop = self.gradePopulation(pop)
            self.sortGradedPopulation(pop)

            self.plotNBest(pop, 10)

            pop = self.selectBest(pop)
            pop = self.reprodPopulation(pop, self.pop_size)
            self.mutatePopulation(pop)   
        end_time = time.time()
        dur = end_time - init_time
        print(RED + "\nGENETIC OPTIMIZATION FINISHED")
        print("Duration : {:.1f} s".format(dur) + DEFAULT)
        self.genLog = np.array(self.genLog)
        return 

    # Analysis methods
    def plotParamHist(self, gradedPop):
        samples = [[]]*len(self.paramTypes)
        for indiv in gradedPop:
            for k in range(len(indiv)):
                samples[k].append(indiv[1][k].value)
        
        for i in range(len(samples)):
            plt.figure()
            plt.hist(samples[i])
            plt.title(self.paramNames[i] + ' histogram for current gen')
        plt.show()


    def plotNBest(self, pop, n):
        for i in range(min(n, len(pop))):
            print()
            print(Fore.MAGENTA + "#" + str(i) + Fore.RESET + " => " + Fore.GREEN + str(int(pop[i][0])) + Fore.RESET)
            self.printParamsInstance(pop[0][1])






