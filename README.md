# path planning
A collection of tools for planning path and collision checking for Dorna robotic robots.

## Requirements
- **CMake** ≥ 3.21  
- **vcpkg** for dependencies (OMPL, FCL, Eigen3, URDFDOM, pybind11)  
- **Python** 3.8–3.12 (64-bit, from [python.org](https://www.python.org/downloads/))  

Make sure Python was installed with *Development headers* (it should include `include/Python.h` and `libs/pythonXY.lib`).

---

## Build Instructions

### 1. Clone repo and submodules
For Linux:

```powershell

sudo apt update
sudo apt install pkg-config autoconf libtool intltool automake autoconf-archive gettext



#install vcpkg and dependencies
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
echo "export VCPKG_ROOT=\"$(pwd)\"" >> ~/.bashrc
echo 'export PATH="$VCPKG_ROOT:$PATH"' >> ~/.bashrc
source ~/.bashrc

vcpkg install ompl
vcpkg install urdfdom
vcpkg install urdfdom-headers
vcpkg install fcl
vcpkg install pybind11

#optional: clearing unnessary vcpkg files:
rm -rf downloads
rm -rf packages
rm -rf buildtrees


#installing py-fcl and urchin
sudo apt install -y libfcl-dev libccd-dev libeigen3-dev liboctomap-dev

git clone https://github.com/BerkeleyAutomation/python-fcl.git
cd python-fcl
sudo python3 setup.py install
sudo pip3 install urchin --break-system-packages



# downloading and building the library
cd ~
git clone https://github.com/dorna-robotics/path_planning.git
cd path_planning

cmake --preset rpi-arm64
cmake --build --preset build-rpi64 -j
sudo cmake --install build/rpi-arm64

```

How to update:
```powershell

cd path_planning
git pull

rm -rf build/

cmake --preset rpi-arm64
cmake --build --preset build-rpi64 -j
sudo cmake --install build/rpi-arm64

```

## Example

###Initialization
```python

import numpy as np
import os 

from path_planning import Planner

# Define a shape dict (box example)
scene = []
load = []

for i in range(4):
    t = i * np.pi / 2.0 + np.pi / 4.0
    l = 0.38
    scene.append(Planner.create_cube( [l*np.cos(t),l*np.sin(t),0, 0, 0, 0],[0.05, 0.05, 0.8] ))

load = [Planner.create_cube([0,0,0.03,0,0,0], [0.01,0.01,0.05])  ]

planner = Planner(    
    scene         = scene,
    load          = load,
    tool          = [0,0,0,0,0,0],
    base_in_world = [0,0,0,0,0,0],
    frame_in_world= [0.0,0.0,0.0, 0,0,0],
    aux_dir       = [ [1,0,0], [0,0,0] ]
    )
```

Checking collision in a special joint configuration:

```python
res = planner.check_collision(joint=[0,0,-169,0,0,0]) #return list of links that collide
```

Planning collision aware motion between start and goal joint configuration:

```python
res = planner.plan(start=[0,0,0,0,0,0,0,0], goal=[-170,0,0,0,0,0,0,0]) #return the path as a list of joint values
```
