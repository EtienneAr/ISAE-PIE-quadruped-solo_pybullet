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
* matplotlib
* multiprocessing
* numpy
* PyBullet ( [latest version here](https://github.com/bulletphysics/bullet3) )

## Quickstart
To run a demo simulation, you can simply run the following command in a terminal :
```shell
python3 isae/scripts/run_simulation.py 
```
The `run_simulation.py` script contains hand-written parameters that are passed to a `footTrajectoryController` and a `walkSimulation` instances. The first one is customizable and basically describes the parameters of the walking behavior we want to test. The second one handles parameters like data logs, simulation duration, and also has methods for data visualization.

**TODO** : include more examples in `run_simulation`.

## Code structure
All the code can be found in the `isae` folder. Here is a quick overview of the folders it contains :
* `control` : contains the code related to the control of the robot based on predefined parameters.
* `optim` : contains the optimization code : our implementation is based on a custom genetic algorithm.
* `scripts` : contains the executable scripts that run a simulation or a whole optimization process.
* `sim_control` : contains the Bullet-related part of the code, i.e. the feedback of data to the controller and the interpretation of the commands as robot moves in the simulated environment.
* `tools` : various pieces of useful code (inverse kinematics model, video recorder).

## Design and optimize your own walk !
In this part, we explain how to use the existing controller and simulation classes to try and design a new walking behavior. It is not extensive and will not cover every detail of the implementation, but hopefully it will give you enough hints about how the code works to dig deeper in the possibiities by yourself.

With this objective in mind, let's see how to adapt the `run_simulation.py` (hand-crafted or pre-optimized walk displayed in simulation) and `run_genAlg.py` (walk optimization) scripts. We will start by presenting the different components (as classes) that come together in these scripts.

### The trajectory class
Before defining a controller, we have to think about the way we want to define the feet trajectories. A few different approaches have been tried out, but the code has also been thought to enable you to easily add a new implementation. The existing trajectory classes can be found under `isae/tools`.

### The controller class
The existing controller classes can be found under `isae/control`. In order to be passed to the simulation handler, they need to implement the `c` method (not the best naming choice here). Its abstract definition is the following :
```python
def c(self, q, q_dot, time, dt): 
    # compute a goal state q_ref
    # compute desired joints torques
    torques = PD(np.array(q_ref), np.zeros((8, 1)), q[7:], q_dot[6:], dt, self.Kp, self.Kd, self.sat)
    return torques
```
Here, `q` and `q_dot` represent the current state of the robot, i.e. the base and joints positions and velocities. From this current state, a goal state `q_ref` is computed and the difference between `q` and `q_ref` is converted to pseudo-motor commands with a PD model. This method is called once per simulation update, which ensures that the simulated motors always receive a well-formated command.\
*Note :* It might not be clear how `q_ref` is computed, but for us, it basically consisted in reading the feet trajectories we wanted and computing the joints positions for each foot thanks to the inverse kinematic model.

**Existing controllers**
