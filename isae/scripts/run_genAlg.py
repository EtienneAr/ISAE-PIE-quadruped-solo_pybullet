#!/usr/bin/python3

# coding: utf8
import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

from isae.optim.genAlg import *
from isae.optim.multiprocessGenAlg import *
from datetime import datetime
from isae.control.footTrajController import *
from isae.control.footTrajControllerV2 import *
from functools import partial 
from isae.control.noiser import *

BLUE = "\033[34m"
GREEN = "\033[32m"
DEFAULT = "\033[39m"
YELLOW = "\033[33m"
CYAN = "\033[36m"
RED = "\033[91m"

#GA = geneticAlgorithm()
GA = multiprocessGeneticAlgorithm()

GA.pop_size = 4
GA.n_gen = 5
GA.grade_index = 3

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

    footTraj1 = footTrajectory([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]])
    footTraj2 = footTrajectory(         footTraj1.points           )
    footTraj3 = footTrajectory(         footTraj1.points           )
    footTraj4 = footTrajectory(         footTraj1.points           )
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
    simInstance.setLoopParams(pyb_gui, duration, leg)
    simInstance.setController(robotController)

    return simInstance

def paramToSim_Bh_Traj(paramsInstance):
    # COMMENT FAIRE??

    # OPTIMIZE BODY HEIGHTS FOR EXAMPLE
    bh0 = paramsInstance[0].value
    bh1 = paramsInstance[1].value
    trajPoints0 = paramsInstance[2].value
    legsOffsets = paramsInstance[3].value

    # Loop parameters 
    pyb_gui = False
    duration = 8
    period = 1.4

    #period = 1.5
    #offsets = [0.5,0.,0.5,0.]
    offsets = legsOffsets
    bodyHeights = 2*[bh0] + 2*[bh1]

    footTraj1 = footTrajectory(           trajPoints0              )
    footTraj2 = footTrajectory(         footTraj1.points           )
    footTraj3 = footTrajectory(         footTraj1.points           )
    footTraj4 = footTrajectory(         footTraj1.points           )
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
    simInstance.setLoopParams(pyb_gui, duration, leg)
    simInstance.setController(robotController)

    return simInstance

def paramToSim_T_cyclePhase_loffs(paramsInstance):

    T = paramsInstance[0].value
    xPhase = paramsInstance[1].value
    yPhase = paramsInstance[2].value
    #legsOffsets = paramsInstance[3].value
    legsOffsets = [0.5,0.5,0.,0.]

    # Loop parameters 
    pyb_gui = False
    duration = 8
    period = T

    Kp = 8
    Kd = 0.2

    offsets = legsOffsets
    bodyHeights = 2*[1.7] + 2*[1.5]

    footTraj1 = footTrajectory([[-0.5,0],[0.1,1.1],[0.5,0],[-0.5,0]])
    footTraj2 = footTrajectory(         footTraj1.points           )
    footTraj3 = footTrajectory(         footTraj1.points           )
    footTraj4 = footTrajectory(         footTraj1.points           )

    def lerpCyclePhase(phase, xVal=[0.5], yVal=[0.5]):
        phase = phase%1.
        
        xVal.append(1)
        xVal.append(0)
        yVal.append(1)
        yVal.append(0)
        
        for i in range(len(xVal) - 1):
            if phase < xVal[i]:
                return yVal[i-1] + (yVal[i] - yVal[i-1])*(1 - (xVal[i] - phase)/(xVal[i] - xVal[i-1]))
    
    setXVal = [xPhase]
    setYVal = [yPhase]

    robotController = footTrajControllerV2(bodyHeights, leg, sols, trajs, offsets, period, partial(lerpCyclePhase,xVal=setXVal, yVal=setYVal), Kp, Kd, 3 * np.ones((8, 1)))
    
    simInstance = gradedSimulation()
    simInstance.setLoopParams(pyb_gui, duration, leg)
    simInstance.setController(robotController)

    return simInstance

def paramToSim_Bh_EtienneCustom(paramsInstance):
    paramValue = list(map(lambda v : v.toArray(), paramsInstance))

    pyb_gui = False
    duration = 10
    period = paramValue[8]

    bodyHeights = [paramValue[9]] * 4

    footTraj1 = customTrajectory(paramValue[0], paramValue[1], paramValue[2], paramValue[3], paramValue[4], paramValue[5], paramValue[6], paramValue[7], 0)
    footTraj2 = customTrajectory(paramValue[0], paramValue[1], paramValue[2], paramValue[3], paramValue[4], paramValue[5], paramValue[6], paramValue[7], 0.5)
    footTraj3 = customTrajectory(paramValue[0], paramValue[1], paramValue[2], paramValue[3], paramValue[4], paramValue[5], paramValue[6], paramValue[7], 0.75)
    footTraj4 = customTrajectory(paramValue[0], paramValue[1], paramValue[2], paramValue[3], paramValue[4], paramValue[5], paramValue[6], paramValue[7], 0.25)
    trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

    leg = Leg(1,1)
    sols = [False, False, True, True]
   
    Kp = 8
    Kd = 0.2

    robotController = footTrajController(bodyHeights, leg, sols, trajs, period, Kp, Kd, 3 * np.ones((8, 1)))
    noiseController = noiseIn_noiseOut(robotController, 1, positionNoise=0.05, velocityNoise=0, torqueNoise=0.05)


    simInstance = gradedSimulation()
    simInstance.setLoopParams(pyb_gui, duration, leg)
    simInstance.setController(noiseController)

    return simInstance

#GA.setParamToSim(paramToSim_Bh_Traj)
GA.setParamToSim(paramToSim_T_cyclePhase_loffs)
GA.setParamToSim(paramToSim_Bh_EtienneCustom)

'''
# params : bh1, bh2, Kp, Kd, period
#paramTypes = ["scalar", "scalar","scalar","scalar","scalar"]
#paramArgs = [[0.8,1.7],[0.8,1.7],[4,20],[0,5],[0.5,4]]
#paramNames = ["BH0", "BH1", "Kp", "Kd", "T"]

# params : triangles summits for traj
paramTypes = ["2dPoint","2dPoint","2dPoint"]
paramArgs = [[[-1.,0],[0,0.2]] , [[-1.,1],[0,1.5]] , [[0,1],[0,0.2]]]

# params : bh1, bh2, footTraj
#paramTypes = ["scalar", "scalar", "ptFtTraj"]
#paramArgs = [[1.2,1.7],[1.2,1.7],[[-1,1.],[0,1.2],[3,4]]]
#paramNames = ["BH0", "BH1", "FootTraj"]

# params : period, xPhase, yPhase, legsOffsets
paramTypes = ["scalar", "scalar", "scalar"] #, "legsOffsets"]
paramArgs = [[0.5,2],[0.1,0.9],[0.1,0.9]] #, 0.6]
paramNames = ["T", "xPhase", "yPhase"] #, "legsOffsets"]

# params : bh1, bh2, footTraj, legsOffsets
#paramTypes = ["scalar", "scalar", "ptFtTraj", "legsOffsets"]
#paramArgs = [[1.2,1.7],[1.2,1.7],[[-1,1.],[0,1.2],[3,4]], 0.6]
#paramNames = ["BH0", "BH1", "FootTraj", "legsOffsets"]

paramTypes = ["scalar", "scalar", "ptFtTraj"]
paramArgs = [[1.2,1.7],[1.2,1.7],[[-1,1.],[0,1.2],[3,4]] ]
paramNames = ["BH0", "BH1", "FootTraj"]
'''

# params : STEP : [ length, height, top_dx, end_dX, end_dy, middle_dx, middle_dy, onGroundPhase] , period , bodyHeight
paramTypes = ["scalarBinary"] * 10
paramArgs = [   [ 0.1, 1.0],  #length
                [ 0.5, 1.5],  #height
                [-0.5, 0.5],  #top_dx
                [ 0.0, 0.2],  #end_dx
                [ 0.0, 0.2],  #end_dy
                [-0.5, 0.5],  #middle_dx
                [-0.2, 0.2],  #middle_dy
                [ 0.5, 1.0],  #onGroundPhase
                [ 0.4, 1.0],  #period
                [ 1.0, 2.0],  #bodyHeight
                ]
paramNames = ["length", "height", "top_dx", "end_dX", "end_dy", "middle_dx", "middle_dy", "onGroundPhase ", "preriod", "bodyHeight"]


GA.setParamTypes(paramTypes)
GA.setParamArgs(paramArgs)
GA.setParamNames(paramNames)

#GA.runOptim(fromIndiv = [ [1.5,1.5,np.array([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]]), [0.,0.5,0.,0.5]] ,
#                          [1.5,1.5,np.array([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]]), [0.,0.,0.5,0.5]] ,
#                          [1.5,1.5,np.array([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]]), [0.5,0.,0.5,0.]],
#                          [1.5,1.5,np.array([[-0.6,0],[-0.0,0.9], [0.6,0], [-0.6,0]]), [0.5,0.,0.,0.5]]])
#prevGenLog = np.load("optim_logs/optim_gen30_pop60_bh_traj_offs.npy", allow_pickle=True)
#lastGen = prevGenLog[-1]
#print(len(lastGen))
#GA.runOptim(fromPop = lastGen)
GA.runOptim()
genLog = np.array(GA.genLog)

#date = datetime.now()
#np.save("optim_logs/optim_"+ date.strftime("%d_%m_%Y_%H:%M:%S") + "_log.npy", genLog, allow_pickle=True)
#print(genLog)

#for k in range(GA.n_gen):
#    pop = np.array(GA.genLog)[k,:]
#    hist = np.histogram(pop[:,0])
#    plt.hist(hist, label="Gen {}".format(k), alpha = 0.5)

n_best = 2
best_per_gen = np.array(GA.genLog)[:,:,0:n_best]

#for i in range(len(best_per_gen)):
#    print(RED + "\n==========================================" + DEFAULT)
#    for j in range(n_best):
#        print(BLUE + "Best indiv. {} for gen {}".format(j, i) + DEFAULT)
#        print(best_per_gen[i, 1, j])
#        print(GREEN + "Fitness : {}\n".format(best_per_gen[i, 0, j]) + DEFAULT)
