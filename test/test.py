
import numpy as np
import os 

import path_planning
from path_planning import Planner

# Define a shape dict (box example)
scene = []
load = []

for i in range(4):
    t = i * np.pi / 2.0 + np.pi / 4.0
    l = 380
    scene.append(Planner.create_cube( [l*np.cos(t),l*np.sin(t),0, 0, 0, 0],[50, 50, 800] ))

load = [Planner.create_cube([0,0,30,0,0,0], [10,10,5])  ]
gripper = [Planner.create_cube([0,0,30,0,0,0], [10,10,5])  ]

planner = Planner(    
    scene = scene,
    load          = load,
    tool          = [0,0,0, 0,0,0],
    gripper        = gripper,
    base_in_world = [0,0,0, 0,0,0],
    frame_in_world= [0.0,0.0,0.0, 0,0,0],
    aux_dir       = [ [1,0,0], [0,0,0] ]
    )

print(planner.check_collision([45,0,0,0,0,0]) , " \n \n \n ")
print(planner.plan([0,0,0,0,0,0,0,0],[-170,0,0,0,0,0,0,0]))
