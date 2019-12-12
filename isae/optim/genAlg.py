from isae.optim.genAlgParam import *
from isae.sim_control.gradedSimulation import *

class geneticAlgorithm(object):

    def __init__(self):
        self.paramTypes = [] # possible types : "scalar", "2dPoint"
        self.paramArgs = [] # corresponding args : [minVal, maxVal], [[xMin, xMax],[yMin, yMax]]
        self.paramNames = [] # ex : ["Period", "Kp"]
        self.pop_size = 4
        self.n_gen = 1
        self.best_per_gen = []
        self.paramToSim = None # expects a function paramToSim(paramsInstance) that returns a gradedSimulation with parameters set
                                # this function has to be defined from outside,
                                # which allows to use any simulation parameters as parameters for optimizatiob

    def setParamTypes(self,paramTypes):
        self.paramTypes = paramTypes

    def setParamArgs(self,paramArgs):
        self.paramArgs = paramArgs

    def printParamsInstance(self, paramsInstance):
        for k in range(len(paramsInstance)):
            print(paramsInstance[k].value)

    # Fonction A CHANGER, definie pour l'instant pour
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
                    paramInstance.initRandom()
                    indiv.append(paramInstance)
                if(self.paramTypes[i] == "2dPoint"):
                    paramInstance = GA_2dPoint(self.paramArgs[i])
                    paramInstance.initRandom()
                    indiv.append(paramInstance)
            pop.append(indiv)
        return pop

    def gradePopulation(self, pop):
        BLUE = "\033[34m"
        GREEN = "\033[32m"
        DEFAULT = "\033[39m"
        YELLOW = "\033[33m"

        gradedPop = []
        for i in range(len(pop)):
            indiv = pop[i]
            sim = self.simFromParam(indiv)
            sim.initializeSim()
            sim.runSim()
            # Choice of optimization function in sim.grades list
            gradedPop.append([sim.grades[0], indiv])
            print(BLUE + "Indiv. " + str(i) + " : " + DEFAULT)
            self.printParamsInstance(indiv)
            print(GREEN + "Fitness : " + str(sim.grades[0]) + DEFAULT)
            print('\n')
        return gradedPop
    
    def sortGradedPopulation(self, gradedPop):
        gradedPop.sort(key=lambda p: -p[0])
        #print(gradedPop)

    def selectBest(self, gradedPop, propToKeep = 0.5):
        n = int(propToKeep*len(gradedPop))
        return gradedPop[:n]

    def reprodPopulation(self, gradedPop):
        rand.shuffle(gradedPop)
        newPop = []
        for k in range(len(gradedPop)/2):
            parent1 = gradedPop[k][1]
            parent2 = gradedPop[-1-k][1]
            child1 = []
            child2 = []
            for i in range(len(parent1)):
                cp1, cp2 = parent1[i].mergeWith(parent2[i])
                child1.append(cp1)
                child2.append(cp2)
            newPop.append(parent1)
            newPop.append(parent2)
            newPop.append(child1)
            newPop.append(child2)

        return newPop
    
    def mutatePopulation(self,gradedPop, probaMut):
        for indiv in gradedPop:
            for k in range(len(probaMut)):
                if rand.random() < probaMut[k]:
                    indiv[k] = indiv[k].mutate()
        return

    def runOptim(self):
        CYAN = "\033[36m"
        DEFAULT = "\033[39m"

        bests = []
        pop = self.initPopulation()
        for k in range(self.n_gen):
            print("\n\n")
            print(CYAN + "GEN " + str(k) + DEFAULT)
            pop = self.gradePopulation(pop)
            self.sortGradedPopulation(pop)
            print("Best params : ")
            self.printParamsInstance(pop[0][1])
            bests.append(pop[0])
            pop = self.selectBest(pop)
            pop = self.reprodPopulation(pop)
            self.mutatePopulation(pop, [0.1]*len(self.paramTypes))

        return bests

    # Analysis methods




