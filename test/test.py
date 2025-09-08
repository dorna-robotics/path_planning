
import numpy as np
import os 

import path_planning
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

print(planner.check_collision([0,0,-169,0,0,0]) , " \n \n \n ")
print(planner.plan([0,0,0,0,0,0,0,0],[-170,0,0,0,0,0,0,0]))
