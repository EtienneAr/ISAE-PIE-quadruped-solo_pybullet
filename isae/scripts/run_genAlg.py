# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

from isae.optim.genAlg import *
from isae.optim.multiprocessGenAlg import *

#GA = geneticAlgorithm()
GA = multiprocessGeneticAlgorithm()

GA.pop_size = 4
GA.n_gen = 5

def paramToSim_Bh_KpKd_T(paramsInstance):
    # COMMENT FAIRE??

    # OPTIMIZE BODY HEIGHTS FOR EXAMPLE
    bh0 = paramsInstance[0].value
    bh1 = paramsInstance[1].value
    Kp = paramsInstance[2].value
    Kd = paramsInstance[3].value
    period = paramsInstance[4].value

    # Loop parameters 
    pyb_gui = False
    duration = 8

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

def paramToSim_Bh_Traj(paramsInstance):
    # COMMENT FAIRE??

    # OPTIMIZE BODY HEIGHTS FOR EXAMPLE
    bh0 = paramsInstance[0].value
    bh1 = paramsInstance[1].value
    trajPoints0 = paramsInstance[2].value

    # Loop parameters 
    pyb_gui = False
    duration = 8
    period = 1.2

    #period = 1.5
    offsets = [0.5,0.,0.5,0.]
    bodyHeights = 2*[bh0] + 2*[bh1]

    footTraj1 = footTrajectory(           trajPoints0              , phaseOffset = offsets[0])
    footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[1])
    footTraj3 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[2])
    footTraj4 = footTrajectory(         footTraj1.points           , phaseOffset = offsets[3])
    trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

    leg = Leg(1,1)
    sols = [False, False, True, True]
    #sols = [True, True, False, False]
    #sols = [False, False, False, False]
    #sols = [True, True, True, True]
    
    Kp = 8
    Kd = 0.2

    robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))

    simInstance = gradedSimulation()
    simInstance.setLoopParams(pyb_gui, duration)
    simInstance.setController(robotController)
    simInstance.setTrajectoryParams(leg, period, trajs, bodyHeights)

    return simInstance

GA.setParamToSim(paramToSim_Bh_Traj)

# params : bh1, bh2, Kp, Kd, period
paramTypes = ["scalar", "scalar","scalar","scalar","scalar"]
paramArgs = [[0.8,1.7],[0.8,1.7],[4,20],[0,5],[0.5,4]]
paramNames = ["BH0", "BH1", "Kp", "Kd", "T"]

# params : triangles summits for traj
#paramTypes = ["2dPoint","2dPoint","2dPoint"]
#paramArgs = [[[-1.,0],[0,0.2]] , [[-1.,1],[0,1.5]] , [[0,1],[0,0.2]]]

# params : bh1, bh2, footTraj
paramTypes = ["scalar", "scalar", "ptFtTraj"]
paramArgs = [[0.8,1.7],[0.8,1.7],[[-1,1.],[0,1.5],[3.,4]] ]
paramNames = ["BH0", "BH1", "FootTraj"]

GA.setParamTypes(paramTypes)
GA.setParamArgs(paramArgs)
GA.setParamNames(paramNames)

best_per_gen = GA.runOptim()
best_fit_per_gen = [indiv[0] for indiv in best_per_gen]

#plt.plot(best_fit_per_gen)
#plt.show()

print("\n########### \n Best param : ")
GA.printParamsInstance(best_per_gen[0][1])
print("\n\nFitness : {}\n###########".format(best_fit_per_gen[0]))

