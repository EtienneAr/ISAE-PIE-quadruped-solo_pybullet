# solo-pybullet
**Simulation and Controller code for Solo Quadruped**

This repository offers an environment to simulate different controllers on the Quadruped robot **Solo**.

You can implement your controller on the *controller.py* file and call your control function in the main program *main.py* by replacing the `c(...)` function in the loop.

## Installation

To install [Pinocchio](https://github.com/stack-of-tasks/pinocchio/), the urdf and meshes of the **Solo** Quadruped,
the [Gepetto Viewer](https://github.com/gepetto/gepetto-viewer-corba) and their python bindings:

```bash
sudo tee /etc/apt/sources.list.d/robotpkg.list <<EOF
deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub $(lsb_release -cs) robotpkg
deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg
EOF
sudo apt update -qqy && sudo apt install -qqy robotpkg-py35-{pinocchio,example-robot-data,qt4-gepetto-viewer-corba}
```

To install PyBullet:
`pip install --user pybullet` (Python 2)
`pip3 install --user pybullet` (Python 3)


## How to start the simulation
There are currently 2 options to run the simulation.
- option 1 : run solo_pybullet as python module with command line arguments (see `solo_pybullet/__main__.py` for the arguments sequence), ex : `python -m solo_pybullet True False 1.5 0.6 0.8 0 1.57 1.57 0 -0.5 0 0.5 0 0 1 8 0.1`
- option 2 : run the simulation script with hard-coded parameters (see the `isae/sim_control/walkSimulation.py` class and the `isae/scripts/run_simulation.py` script), with the following line : `python isae/scripts/run_simulation.py`

The simulation in its current state should be completely handled by the pybullet engine and this repo, and should not interact with the Pinocchio packages except to load Solo URDF file.
`
