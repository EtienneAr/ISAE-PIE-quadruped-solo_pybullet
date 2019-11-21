# -*- coding: utf-8 -*-


import os, sys
from sys import argv
sys.path.insert(0, os.getcwd()) # adds current directory to python path

from isae.sim_control.walkSimulation import *

import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui

app = QtGui.QApplication([])
import pyqtgraph.parametertree.parameterTypes as pTypes
from pyqtgraph.parametertree import Parameter, ParameterTree, ParameterItem, registerParameterType

# Create simulation instance
walkSim = walkSimulation()

# Define GUI parameters
params = [
    {'name': 'PyBullet simulation parameters', 'type':'group', 'children': [
        {'name': 'GUI', 'type': 'bool', 'value': True},
        {'name': 'Duration (s)', 'type': 'float', 'value': 10, 'step': 1, 'limits': (3, 30)},       
    ]},

    {'name': 'Controller parameters', 'type':'group', 'children': [
        {'name': 'K_p', 'type': 'float', 'value': 10, 'step': 1, 'limits': (1, 100)}, 
        {'name': 'K_d', 'type': 'float', 'value': 0.2, 'step': 0.1, 'limits': (0, 50)},  
        {'name': 'Saturation', 'type': 'float', 'value': 3, 'step': 0.1, 'limits': (0, 50)},       
    ]},

    {'name': 'Behavior parameters', 'type':'group', 'children': [
        {'name': 'Movement period (s)', 'type': 'float', 'value': 1, 'step': 0.1, 'limits': (0.2, 5)}, 
        {'name': 'Legs phases offsets', 'type':'group', 'children': [
            {'name': 'Leg0 phase', 'type': 'float', 'value': 0.0, 'step': 0.1, 'limits': (0., 0.99)}, 
            {'name': 'Leg1 phase', 'type': 'float', 'value': 0.5, 'step': 0.1, 'limits': (0., 0.99)}, 
            {'name': 'Leg2 phase', 'type': 'float', 'value': 0.5, 'step': 0.1, 'limits': (0., 0.99)}, 
            {'name': 'Leg3 phase', 'type': 'float', 'value': 0., 'step': 0.1, 'limits': (0., 0.99)}, 
        ]},
        {'name': 'Body heights wrt. legs', 'type':'group', 'children': [
            {'name': 'Leg0 height', 'type': 'float', 'value': 1.0, 'step': 0.1, 'limits': (0, 2)}, 
            {'name': 'Leg1 height', 'type': 'float', 'value': 1.0, 'step': 0.1, 'limits': (0, 2)}, 
            {'name': 'Leg2 height', 'type': 'float', 'value': 1.0, 'step': 0.1, 'limits': (0, 2)}, 
            {'name': 'Leg3 height', 'type': 'float', 'value': 1.0, 'step': 0.1, 'limits': (0, 2)}, 
        ]},

    ]},
]

## Create tree of Parameter objects
p = Parameter.create(name='params', type='group', children=params)

t = ParameterTree()
t.setParameters(p, showTop=False)
t.setWindowTitle('Simulation parameters GUI')

# Button callback
def btnpressed():
      print("Clicked the button!")
      print(p.children()[0].children()[0].value())

# Read GUI parameters and assign them to the simulation
def updateParametersCallback():

    # Read parameters from GUI

    # PyBullet parameters
    enableGUI = p.children()[0].children()[0].value()
    duration = p.children()[0].children()[1].value()

    # Controller parameters
    leg = Leg(1,1)

    K_p = p.children()[1].children()[0].value()
    K_d = p.children()[1].children()[1].value()
    sat = p.children()[1].children()[2].value()

    # Behavior parameters
    period = p.children()[2].children()[0].value()
    phases_offsets = [
        p.children()[2].children()[1].children()[0].value(),
        p.children()[2].children()[1].children()[1].value(),
        p.children()[2].children()[1].children()[2].value(),
        p.children()[2].children()[1].children()[3].value()
    ]
    bodyHeights = [
        p.children()[2].children()[2].children()[0].value(),
        p.children()[2].children()[2].children()[1].value(),
        p.children()[2].children()[2].children()[2].value(),
        p.children()[2].children()[2].children()[3].value()
    ]

    # Feet trajectories
    footTraj1 = footTrajectory([[-0.5,0],[0.,.5], [0.5,0], [-0.5,0]], phaseOffset = phases_offsets[0])
    footTraj2 = footTrajectory(         footTraj1.points           , phaseOffset = phases_offsets[1])
    footTraj3 = footTrajectory([[-0.5,0],[0.,.5], [0.5,0], [-0.5,0]], phaseOffset = phases_offsets[2])
    footTraj4 = footTrajectory(         footTraj3.points           , phaseOffset = phases_offsets[3])

    trajs = [footTraj1, footTraj2, footTraj3, footTraj4]

    # Assign parameters to the simulation
    walkSim.setLoopParams(enableGUI, duration)
    walkSim.setControllerParams(leg, K_p, K_d)
    walkSim.setTrajectoryParams(period, trajs, bodyHeights)

    print("Updated simulation parameters")

# Run the simulation
def runSimCallback():
    # Run sim
    walkSim.runSim()

# plots feet trajectories and contacts with the ground
def plotResultsCallback():
    walkSim.plotFeetAvgPos()

    walkSim.plotContactPoints()
    walkSim.plotBasePos()
    plt.show()



# "Update parameters" button widget
updateButton = QtGui.QPushButton("Update parameters")
updateButton.setCheckable(False)
updateButton.toggle()
updateButton.clicked.connect(updateParametersCallback)

# "Run simulation" button widget
runButton = QtGui.QPushButton("Run simulation")
runButton.setCheckable(False)
runButton.toggle()
runButton.clicked.connect(runSimCallback)

# "Plot results" button widget
plotButton = QtGui.QPushButton("Plot results")
plotButton.setCheckable(False)
plotButton.toggle()
plotButton.clicked.connect(plotResultsCallback)

# Define GUI window and grid layout
win = QtGui.QWidget()

layout = QtGui.QGridLayout()
win.setLayout(layout)

# Add widgets to the layout
layout.addWidget(QtGui.QLabel("This GUI helps with defining the simulation parameters for PyBullet"), 0,  0, 1, 2)
layout.addWidget(t, 1, 0, 1, 1)
layout.addWidget(updateButton, 2, 0, 1, 1)
layout.addWidget(runButton, 3, 0, 1, 1)
layout.addWidget(plotButton, 4, 0, 1, 1)

win.show()
win.resize(400,600)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
    walkSim.runSim()