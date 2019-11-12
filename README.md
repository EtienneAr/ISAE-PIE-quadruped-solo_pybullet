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
`pip install --user pybullet`

## How to start the simulation
If you are on the master branch :\
launch `gepetto-gui`, then `python -m solo_pybullet`

If you are on the Etienne or Thibault branch :\
launch `gepetto-gui`, then `python -m solo_pybullet guiOn rtSimuOn bodyHeight stepPeriod stepLen phaseOffset_1 phaseOffset_2 phaseOffset_3 phaseOffset_4 point0_X point0_Y point1_X point1_Y point2_X point2_Y Kp Kd`\
Ex : `python -m solo_pybullet True False 1.5 0.6 0.8 0 1.57 1.57 0 -0.5 0 0.5 0 0 1 8 0.1`
`
