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


git clone https://github.com/microsoft/vcpkg.git
cd vcpkg
./bootstrap-vcpkg.sh
echo 'export VCPKG_ROOT="$HOME/vcpkg"' >> ~/.bashrc
echo 'export PATH="$VCPKG_ROOT:$PATH"' >> ~/.bashrc
source ~/.bashrc

vcpkg install ompl
vcpkg install urdfdom
vcpkg install urdfdom-headers
vcpkg install fcl
vcpkg install pybind11

cd ~
git clone https://github.com/dorna-robotics/path_planning.git
cd path_planning

cmake --preset rpi-arm64   -D Python3_EXECUTABLE="$(command -v python3)"   -D Python3_INCLUDE_DIR="/usr/include/python3.11"   -D Python3_LIBRARY="/usr/lib/aarch64-linux-gnu/libpython3.11.so"

cmake --build build/rpi-arm64 --config Release -j

sudo cmake --install build/rpi-arm64 --config Release

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
    scene = scene,
    load          = load,
    tool          = [0,0,0, 0,0,0],
    base_in_world = [0,0,0, 0,0,0],
    frame_in_world= [0.0,0.0,0.0, 0,0,0],
    aux_dir       = [ [1,0,0], [0,0,0] ]
    )
```

Checking collision in a special joint configuration:

```python
planner.check_collision([0,0,-169,0,0,0])
```

Planning collision aware motion between start and goal joint configuration:

```python
planner.plan([0,0,0,0,0,0,0,0],[-170,0,0,0,0,0,0,0])
```