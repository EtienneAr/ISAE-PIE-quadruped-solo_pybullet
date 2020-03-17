# solo-pybullet
**In short**

The content of this repository was developed in the context of an academic group project. As a team of 6 ISAE-Supaero students, we worked for the LAAS-CNRS robotics lab to extend their previous repo of simulation and control for the SOLO robot.
The team was composed of Etienne ARLAUD, Ethan CHERKI, Thomas CORBERES, Thibault NOËL, Jean-Lou QUETIN and Marion VALETTE. 

This repo mainly serves the following purposes :
* providing a relevant simulation environment for SOLO (ease of use, data extraction)
* providing simulation-based controllers to easily test walking behaviors
* providing optimization tools to automatically search for viable walking behaviors

## Prerequisites
In order to properly use this package, you need to run Python3 and have the following libraries installed :
* numpy
* matplotlib
* PyBullet ( [latest version here](https://github.com/bulletphysics/bullet3) )

## Quickstart
To run a demo simulation, you can simply run the following command in a terminal :
```shell
python3 isae/scripts/run_simulation.py 
```

## Code structure
All the code can be found in the `isae` folder. Here is a quick overview of the folders it contains :
* `control` : contains the code related to the control of the robot based on predefined parameters.
* `optim` : contains the optimization code : our implementation is based on a custom genetic algorithm.
* `scripts` : contains the executable scripts that run a simulation or a whole optimization process.
* `sim_control` : contains the Bullet-related part of the code, i.e. the feedback of data to the controller and the interpretation of the commands as robot moves in the simulated environment.
* `tools` : various pieces of useful code (inverse kinematics model, video recorder).
