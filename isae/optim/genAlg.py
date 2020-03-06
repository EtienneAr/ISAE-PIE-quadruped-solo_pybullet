from isae.optim.genAlgParam import *
from isae.sim_control.gradedSimulation import *
from colorama import Fore

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
                                # which allows to use any simulation parameters as parameters for optimizatiob

    def setParamTypes(self,paramTypes):
        self.paramTypes = paramTypes

    def setParamArgs(self,paramArgs):
        self.paramArgs = paramArgs

    def setParamNames(self,paramNames):
        self.paramNames = paramNames

    def printParamsInstance(self, paramsInstance):
        for k in range(len(paramsInstance)):
            print(str(paramsInstance[k].value) + ", \t#" + self.paramNames[k])

    def getIndivArray(self, indiv):
        return np.array([p.toArray() for p in indiv])

    # Fonction A CHANGER, definie pour l'insparamArgs
    # des cas specifiques a chaque fois

    def setParamToSim(self, paramToSim):
        self.paramToSim = paramToSim

    def simFromParam(self, paramsInstance):
        return self.paramToSim(paramsInstance)

    """
    # params : bh1, bh2, Kp, Kd, period
    def simFromParam(self, paramsInstance):
        # COMMENT FAIRE??

        # OPTIMIZE BODY HEIGHTS FOR EXAMPLE
        bh0 = paramsInstance[0].value
        bh1 = paramsInstance[1].value
        Kp = paramsInstance[2].value
        Kd = paramsInstance[3].value
        period = paramsInstance[4].value

        # Loop parameters 
        pyb_gui = False
        duration = 10

        #period = 1.5
        offsets = [0.5,0.,0.5,0.]
        bodyHeights = 2*[bh0] + 2*[bh1]

        footTraj1 = footTrajectory([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]], phaseOffset = offsets[0])
        footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
        footTraj3 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[2])
        footTraj4 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[3])
        trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

        leg = Leg(1,1)
        sols = [False, False, True, True]
        #sols = [True, True, False, False]
        #sols = [False, False, False, False]
        #sols = [True, True, True, True]
        
        #Kp = 8
        #Kd = 0.2

        robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

        simInstance = gradedSimulation()
        simInstance.setLoopParams(pyb_gui, duration)
        simInstance.setController(robotController)
        simInstance.setTrajectoryParams(leg, period, trajs, bodyHeights)

        return simInstance
    """

    """
    # params : t0, t1, t2 triangle summits for the foot traj
    def simFromParam(self, paramsInstance):
        # COMMENT FAIRE??

        # OPTIMIZE triangle points
        t0 = paramsInstance[0].value
        t1 = paramsInstance[1].value
        t2 = paramsInstance[2].value

        # Loop parameters 
        pyb_gui = False
        duration = 10

        period = 1.2
        offsets = [0.5,0.,0.5,0.]
        bodyHeights = 2*[1] + 2*[1.2]

        footTraj1 = footTrajectory([t0,t1,t2,t0], phaseOffset = offsets[0])
        footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
        footTraj3 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[2])
        footTraj4 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[3])
        trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

        leg = Leg(1,1)
        sols = [False, False, True, True]
        #sols = [True, True, False, False]
        #sols = [False, False, False, False]
        #sols = [True, True, True, True]
        
        Kp = 10
        Kd = 0.4

        robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

        simInstance = gradedSimulation()
        simInstance.setLoopParams(pyb_gui, duration)
        simInstance.setController(robotController)
        simInstance.setTrajectoryParams(leg, period, trajs, bodyHeights)

        return simInstance
    """

    def initPopulation(self):
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
                paramInstance.initRandom()
                indiv.append(paramInstance)
            pop.append(indiv)
        return pop

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
    
    def sortGradedPopulation(self, gradedPop):
        gradedPop.sort(key=lambda p: -p[0])
        popParams = np.array(gradedPop)[:,2]
        popFitness = np.array(gradedPop)[:,0]
        self.genLog.append([popFitness, popParams])

    def selectBest(self, gradedPop, propToKeep = 0.33):
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
    
    def mutatePopulation(self,gradedPop):
        for indiv in gradedPop:
            for gene in indiv:
                gene.mutate()

    def runOptim(self):
        
        init_time = time.time()
        CYAN = Fore.CYAN
        RED = Fore.RED
        DEFAULT = Fore.RESET

        pop = self.initPopulation()
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
            self.printParamsInstance(pop[i][1])






