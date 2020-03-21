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
Our implementation requires any trajectory class to implement a `getPos` method, so that they are interchangeable and extendable fairly easily. This method is defined as follows :
```python
def getPos(self, phase): 
    # compute where we are on the trajectory given the phase (frac. of control period)
    currPos = [currX, currY]
    # return the [x,y] pos as a np.array
    return np.array(currPos)
```
The `getPos` method is basically called by the controller at each simulation update, to know the desired position of each foot; the controller has a period attribute which allows it to feed `getPos` with the current 'phase', computed using the period and elapsed simulation time.  

**_IMPORTANT NOTICE_** : all of our trajectories use the length of an actuator as unit (half the total leg length), and are referenced in the ground frame aligned vertically with the hip joint. (*Ex* : a feet position `[x,y] = [0,1]` means that the foot is above the ground at knee-height directly under the hip, while `[x,y] = [0.2,0]` means the foot is on the ground but slightly in front of the hip.)

**Existing trajectories**
- `footTrajectory.pointsTrajectory` : deprecated 
- `footTrajectory.footTrajectory` : stores the foot trajectory as a list of 2D points, read in order.  
  *Arguments* :
  * `points` : python list in format `[[x0,y0],[x1,y1],...,[xn,yn],[x0,y0]]`
- `footTrajectory.customTrajectory` : generates a trajectory from macro parameters (step length, time spent with foot on ground...)  
  *Arguments* :
  * `length`: length of a step
  * `height`: max height of the foot above ground
  * `top_dx`: 
  * `end_dx`:
  * `end_dy`:
  * `middle_dx`:
  * `middle_dy`:
  * `onGroundPhase`: float in `[0,1[`, representing the duration the foot should be in contact with the ground, as a fraction of the period of the leg movement.
  * `phaseOffset`: float in `[0,1[`, representing the delay on the leg this trajectory is attached to, again as a fraction of the control period.
- `footTrajectoryBezier.footTrajectoryBezier` : generates a trajectory as a closed Bezier curve.  
  *Arguments* : 
  * `points` : python list in format `[[x1 y1 x2 y2 x3 y3 x4 y4], [delta_x1 d_y1 d_x2 d_y2 d_x3 d_y3 d_x4 d_y4]]`. The `x,y` arguments are the coordinates for 4 control points, and the `dx,dy` arguments define the 2D derivative at each of these control points. 
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

### The simulation class

### Instanciating the simulation

### The optimization parameter class
**Existing parameters**

### Instanciating the optimizer 
