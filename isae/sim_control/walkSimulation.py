# coding: utf8

# Wrapper class for pybullet simulation with adapted parameters for Solo and our controllers

import time

# PyBullet simulator
import pybullet as p 
# Numpy library
import numpy as np  
# Leg class
from isae.tools.geometry import * 
# footTrajectory class
from isae.tools.footTrajectory import * 
# footTajectory class Bezier Curve
from isae.tools.footTrajectoryBezier import *

# grading class : in optim module, expects grade and getGrade (aka final grade) as class methods

# for 3D plots in matplotlib
from mpl_toolkits.mplot3d import Axes3D 

# Functions to initialize the simulation and retrieve joints positions/velocities
from isae.sim_control.initialization_simulation_pybullet import configure_simulation, getPosVelJoints

from matplotlib import rc
rc('font',**{'family':'serif','serif':['New Century Schoolbook'], 'size'   : 14})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
rc('text', usetex=True)

class walkSimulation(object):
    def __init__(self):

        # Parameters for the main pybullet loop
        # Current simulation step
        self.step = 0
        # Timestep of the simulation
        self.dt = 0.001
        # Total duration of the simulation
        self.duration = 10.
        # Targeted real-time factor (to keep? to delete?)
        self.RTF = 0.
        # pybullet graphical interface
        self.enableGUI = False

        # Trajectories-related parameters

        # Period of the movement 
        self.period = 1.
        # Array of 4 feet trajectories
        self.legsTraj = [] 
        # Body heights wrt. the traj. reference frame during the movement, array of size 4 with values in [0,2[ (unit: leg height)
        self.bodyHeights = [] 

        # Geometry of the leg
        self.Leg = None
        # IK solution to be used, array of boolean of size 4 (see the Geometry class)
        #self.sols = []
        # Kp gain of the controller
        #self.Kp = 0. # 8.
        # Kd gain of the controller
        #self.Kd = 0. # 0.2
        # Saturation of the controller
        #self.sat = float('inf')  # 3 

        # Results to be stored
        # Contacts between the robot and the ground
        self.contactPoints = []
        self.contactsFoot0 = []
        self.contactsFoot1 = []
        self.contactsFoot2 = []
        self.contactsFoot3 = []
        self.otherContacts = []
        self.storeContactPoints = True
        # Feet positions in time
        self.feetPos = []
        self.storeFeetPos = True
        # Base state 
        self.qBase = []
        self.qdotBase = []
        self.store_qBase = True

        # Video parameters
        self.cameraTool = None        

        # Internal variables
        self.robotController = None 
        self.robotId = None
        self.revoluteJointIndices = None
        self.physicsClient = None

    ###
    # Setting attributes (simulation parameters)
    ###
    def setLoopParams(self,enableGUI, duration, legGeometry, RTF=0.):
        self.enableGUI = enableGUI
        self.duration = duration
        self.Leg = legGeometry
        self.RTF = RTF

    #def setControllerParams(self,Leg, sols = 4*[False], Kp = 0, Kd = 0, sat = float('inf')):
    #    self.Leg = Leg
    #    self.sols = sols
    #    self.Kp = Kp*1. # 8.
    #    self.Kd = Kd*1. # 0.2
    #    self.sat = float('inf') # 3

    def setController(self, robotController):
        self.robotController = robotController

    def setCameraTool(self, cameraTool):
        self.cameraTool = cameraTool

    ###
    # Running the simulation
    ###
    def initializeSim(self):
        self.robotId, self.revoluteJointIndices, self.physicsClient = configure_simulation(self.dt, self.enableGUI) # initializes PyBullet client
        #self.robotController = footTrajController(self.bodyHeights, self.Leg, self.sols, self.legsTraj, self.period, self.Kp, self.Kd, 3 * np.ones((8, 1)))

    # Wrapper around pybullet_client.stepSimulation()
    def stepSim(self):
        # Time at the start of the loop
        #t0 = time.clock()
        # Get position and velocity of all joints in PyBullet (free flying base + motors)
        q, qdot = getPosVelJoints(self.robotId, self.revoluteJointIndices, self.physicsClient)

        # Call controller to get torques for all joints
        jointTorques = self.robotController.c(q, qdot, self.step*self.dt, self.dt) # c_walking(q, qdot, dt, solo, i * dt)

        # Set control torques for all joints in PyBullet
        self.physicsClient.setJointMotorControlArray(self.robotId, self.revoluteJointIndices, controlMode=self.physicsClient.TORQUE_CONTROL, forces=jointTorques)

        # Compute one step of simulation
        self.physicsClient.stepSimulation()

        # Duration of the step
        #t_step = time.clock() - t0

        if self.storeFeetPos :
            q_joints = q[7:]
            footpos0 = self.Leg.getFootPos(q_joints[0:2])
            footpos1 = self.Leg.getFootPos(q_joints[2:4])
            footpos2 = self.Leg.getFootPos(q_joints[4:6])
            footpos3 = self.Leg.getFootPos(q_joints[6:])

            self.feetPos.append([footpos0,footpos1,footpos2,footpos3])
        
        if self.store_qBase :
            q_base = q[:7]
            qdot_base = qdot[:7]
            self.qBase.append(q_base)
            self.qdotBase.append(qdot_base)

        if self.storeContactPoints:
            # Store contact points at this time step
            contacts = self.physicsClient.getContactPoints(bodyA = 0) # checks collisions with bodyA=0 (ground plane)
            for point in contacts : 
                self.contactPoints.append([point[5][0], point[5][1], point[4], point[9], self.step]) # stores the x,y position of the contact point, the link (index) in contact with the ground and the normal force

                if (point[4] == 2):# or (point[4] == 2): # link index for foot0 and leg0 bottom part
                    self.contactsFoot0.append(self.step)
                elif point[4] == 5: # link index for foot1
                    self.contactsFoot1.append(self.step)
                elif point[4] == 8: # link index for foot2
                    self.contactsFoot2.append(self.step)
                elif point[4] == 11: # link index for foot3
                    self.contactsFoot3.append(self.step)
                else: # link index for foot1
                    self.otherContacts.append(self.step)
        self.step += 1


    def runSim(self):
        init_sim_date = time.clock()

        # Main loop
        iterations = int(self.duration/self.dt)
        for k in range(iterations):
            #t0 = time.clock()
            self.stepSim()
            if self.cameraTool.recordVideo : self.saveImages(k)

        # Record the video from images saved        
        if self.cameraTool.recordVideo : self.cameraTool.saveVideo()

        # Shut down the PyBullet client
        self.physicsClient.disconnect()

        end_sim_date = time.clock()
        print("##########################################\n"+
            "Simulation finished in " +
            str(end_sim_date - init_sim_date) + " s\n##########################################")
    
    # Synchronizes the stepSim() calls for multiple simulations
    # other_sims : list of simulation that have been initialized, and assumes the same duration for now
    def runSimParallelWith(self, other_sims):
        init_sim_date = time.clock()

        for sim in other_sims :
            sim.initializeSim()
        #self.initializeSim()

        # Main loop
        iterations = int(self.duration/self.dt)
        for k in range(iterations):
            self.stepSim()
            other_sims[0].stepSim()
            #for sim in other_sims :
            #    sim.stepSim()
            
        
        # Shut down pybullet clients
        self.physicsClient.disconnect()
        for sim in other_sims :
            sim.physicsClient.disconnect()
        
        
        end_sim_date = time.clock()
        print("##########################################\n"+
            str(len(other_sims)) + " simulations finished in " +
            str(end_sim_date - init_sim_date) + " s\n##########################################")
            
    
    ###
    # Helpful functions for grading
    ###
    # Return rolling average of array param with a window of size n
    def rollingAvg(self, param, n):
        arr = np.array(param)
        rolling_avg = np.zeros(arr.shape)
        l = len(arr)
        for k in range(n):
            rolling_avg = rolling_avg + np.concatenate((k*[0.],arr[:l-k]))
        rolling_avg /= n
        return rolling_avg
    
    # Return distance between the initial base position (x0,y0) 
    # and the final base position (xf,yf)
    # offset : number of steps to skip before setting (x0,y0)
    def getFinalDistance(self, offset=0):
        self.qBase = np.array(self.qBase)
        x0, y0 = self.qBase[:,0][0+offset], self.qBase[:,1][0+offset]
        xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]

        return np.sqrt((xf-x0)**2 + (yf-y0)**2)[0]
    ###
    # Plotting the results
    ###

    def plotContactPoints(self):
        self.contactPoints = np.array(self.contactPoints)
        
        foot0_timeline = 1.0*np.array([t in self.contactsFoot0 for t in range(self.step)])
        foot1_timeline = 1.0*np.array([t in self.contactsFoot1 for t in range(self.step)])
        foot2_timeline = 1.0*np.array([t in self.contactsFoot2 for t in range(self.step)])
        foot3_timeline = 1.0*np.array([t in self.contactsFoot3 for t in range(self.step)])

        timeline = [i*self.dt for i in range(self.step)]

        print(str(len(self.contactPoints)) + " contact points")
        plt.fill_between(timeline, 3, 3+foot0_timeline, color='b', label="Foot 0 - FL")
        plt.fill_between(timeline, 2, 2+foot1_timeline, color='r', label="Foot 1 - FR")
        plt.fill_between(timeline, 1, 1+foot2_timeline, color='g', label="Foot 2 - RL")
        plt.fill_between(timeline, 0, 0+foot3_timeline, color='y', label="Foot 3 - RR")
        #plt.plot(4. + np.array(foot1_timeline), label="Foot 1")
        #plt.plot(2. + np.array(foot2_timeline), label="Foot 2")
        #plt.plot(np.array(foot3_timeline), label="Foot 3")

        # TO KEEP
        #plt.scatter(self.contactPoints[:,0], self.contactPoints[:,1], c=colors, marker='+',s=25)
        plt.xlabel("$\mathbf{t} (s)$")
        plt.legend()
        plt.title("Contact points with ground")

    def plotContactForces(self):
        self.contactPoints = np.array(self.contactPoints)
        #contacts = self.contactPoints[:,2:5]
        timeline = [i*self.dt for i in range(self.step)]

        force0,force1,force2,force3 = len(timeline)*[0.],len(timeline)*[0.],len(timeline)*[0.],len(timeline)*[0.]
        for p in self.contactPoints:
            if p[2] == 2.:
                force0[int(p[4])] = p[3]
            if p[2] == 5.:
                force1[int(p[4])] = p[3]
            if p[2] == 8.:
                force2[int(p[4])] = p[3]
            if p[2] == 11.:
                force3[int(p[4])] = p[3]

        filter_size = 10
        force0 = self.rollingAvg(force0, filter_size)
        force1 = self.rollingAvg(force1, filter_size)
        force2 = self.rollingAvg(force2, filter_size)
        force3 = self.rollingAvg(force3, filter_size)

        plt.fill_between(timeline, 0, force0, label="Foot 0 - FL", color='b', alpha = 0.3)
        plt.fill_between(timeline, 0, force1, label="Foot 1 - FR", color='r', alpha = 0.3)
        plt.fill_between(timeline, 0, force2, label="Foot 2 - RL", color='g', alpha = 0.3)
        plt.fill_between(timeline, 0, force3, label="Foot 3 - RR", color='y', alpha = 0.3)

        plt.xlabel("$\mathbf{t} (s)$")
        plt.ylabel("$\mathbf{F_n}$ $(N)$")
        plt.title("Contact forces with ground")
        
    def plotFeetAvgPos(self):
        self.feetPos = np.array(self.feetPos)
        
        #0.19 0.1046

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        plt.plot(self.feetPos[:,0,0] + 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,0,1], label='Foot 0 traj.')
        plt.plot(self.feetPos[:,1,0] + 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,1,1], label='Foot 1 traj.')
        plt.plot(self.feetPos[:,2,0] - 0.19, [0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,2,1], label='Foot 2 traj.')
        plt.plot(self.feetPos[:,3,0] - 0.19, [-0.1046]*len(self.feetPos[:,0,0]), self.feetPos[:,3,1], label='Foot 3 traj.')
        plt.legend()
        plt.show()

        # Plot feet trajectories
        #plt.title("Feet average cycle")
    
    def plotBasePos(self):
        self.qBase = np.array(self.qBase)

        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        x0, y0 = self.qBase[:,0][0], self.qBase[:,1][0]
        xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]
        
        filter_size = 1000
        xAvg = self.rollingAvg(self.qBase[:,0,0], filter_size)
        yAvg = self.rollingAvg(self.qBase[:,1,0], filter_size)

        plt.plot(self.qBase[:,0], self.qBase[:,1], label='Base xy traj.')
        plt.plot(xAvg, yAvg, 'r', linewidth=2, label='Avg. base xy traj.')
        plt.plot([x0,xf],[y0,yf], '--', linewidth = 2)

        plt.xlabel("x (m)")
        plt.ylabel("y (m)")
        plt.title("Base [x,y] position")
    
    def plotBaseSpeed(self):
        self.qdotBase = np.array(self.qdotBase)

        filter_size = 500
        xdotAvg = self.rollingAvg(self.qdotBase[:,0,0], filter_size)
        ydotAvg = self.rollingAvg(self.qdotBase[:,1,0], filter_size)
        #fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")

        #xdot0, y0 = self.qBase[:,0][0], self.qBase[:,1][0]
        #xf, yf = self.qBase[:,0][-1], self.qBase[:,1][-1]

        plt.plot(self.qdotBase[:,0,0],self.qdotBase[:,1,0], 'g',linewidth=0.5, label='Base xy speed')
        plt.plot(xdotAvg,ydotAvg, 'r', linewidth=2,label='Avg base speed')
        #plt.plot([x0,xf],[y0,yf], '--', linewidth = 2)
        
        plt.xlabel("$v_x (m.s^{-1})$")
        plt.ylabel("$v_y (m.s^{-1})$")
        plt.title("Base [x,y] speed")

    def plotBaseSpeedXY(self):
        self.qdotBase = np.array(self.qdotBase)

        filter_size = 500
        xdotAvg = self.rollingAvg(self.qdotBase[:,0,0], filter_size)
        ydotAvg = self.rollingAvg(self.qdotBase[:,1,0], filter_size)

        plt.plot(self.qdotBase[:,0,0],'g',linewidth=0.5, label='Base x speed')
        plt.plot(xdotAvg, 'r', linewidth=2,label='Avg base speed')
        plt.legend()
        plt.figure()
        plt.plot(self.qdotBase[:,1,0], 'g',linewidth=0.5, label='Base y speed')
        plt.plot(ydotAvg, 'r', linewidth=2,label='Avg base speed')

        plt.title("Base [x,y] speed")

    def plotBaseAbsXYSpeed(self, filters=[500]):
        self.qdotBase = np.array(self.qdotBase)
        absXYSpeed = np.sqrt(self.qdotBase[:,0,0]**2 + self.qdotBase[:,1,0]**2)
        lw = 0.5

        plt.plot(absXYSpeed,linewidth=lw, label='Base abs speed')

        for s in filters : 
            filter_size = s
            xdotAvg = self.rollingAvg(self.qdotBase[int(s/2):,0,0], filter_size)
            ydotAvg = self.rollingAvg(self.qdotBase[int(s/2):,1,0], filter_size)
            avgAbsXYSpeed = np.sqrt(xdotAvg**2 + ydotAvg**2)

            plt.plot(avgAbsXYSpeed, linewidth=lw, label='Avg base abs speed filter ' + str(s))

        plt.xlabel("t (ms)")
        plt.ylabel("$v_abs (m.s^{-1})$")
        plt.title("Base absolute speed")
        


    def plotTrajBase(self):
        fig = plt.figure()
        #ax = fig.add_subplot(1, 1, 1, axisbg="1.0")
        ax = fig.add_subplot(2,1,1)
        ax.set_xlabel('x_world (m)')
        ax.set_ylabel('y_world (m)')
        self.plotContactPoints()
        self.plotBasePos()

        ax2 = fig.add_subplot(2,1,2)
        self.feetPos = np.array(self.feetPos)
        x_foot_0 = self.qBase[:,0,0] + self.feetPos[:,0,0]
        z_foot_0 = self.qBase[:,2,0] + self.feetPos[:,0,1]
        plt.plot(self.qBase[:,0,0],self.qBase[:,2,0], '-', linewidth=2, label='Base xz traj.')
        plt.plot(x_foot_0,z_foot_0, '-', label='Foot traj. + base mvt.')

        ax.legend()
        ax.grid(True)
        ax2.legend()
        ax2.grid(True)
        plt.show()
    
    def saveImages(self, iteration):
        iteration_begin = int(self.cameraTool.timeBeginning/self.dt)
        nb_iteration_max = int(self.duration/self.dt)
        nb_total_image =  int( (self.duration - self.cameraTool.timeBeginning) * self.cameraTool.fps)
        if iteration > iteration_begin:                  
            if iteration%int(nb_iteration_max/nb_total_image) == 0:
                self.cameraTool.saveImage(self.cameraTool.getRGBAImage())
