from . import core

import numpy as np
import fcl
from dorna2 import pose
from importlib.resources import files, as_file


from . import urdf
from . import node


def check_collision(joint,							#In degrees
					tool=[0,0,0,0,0,0],  		
					load=[],					
					scene=[],					
					base_in_world=[0,0,0,0,0,0],
					frame_in_world=[0,0,0,0,0,0],
					aux_dir=[[0, 0, 0], [0, 0, 0]],
					aux_limit=[[-1,1],[-1,1]]
	):
	

	root_node = node.Node("root")

	urdf_path = res = files("path_planning") / "resources" / "urdf" / "dorna_ta.urdf"

	robot = urdf.urdf_robot(urdf_path, {}, root_node)

	all_visuals = [] #for visualization
	all_objects = [] #to create bvh
	dynamic_objects = [] #to update bvh

	#placing scene objects
	for obj in scene:
		root_node.collisions.append(obj.fcl_object)
		all_objects.append(obj.fcl_object)
		all_visuals.append(obj)

	#placing tool objects
	for obj in load:
		robot.link_nodes["j6_link"].collisions.append(obj)
		robot.all_objs.append(obj)
		robot.prnt_map[id(obj.fcl_shape)] = robot.link_nodes["j6_link"]

	#registering robot objects
	for obj in robot.all_objs:
		dynamic_objects.append(obj)
		all_objects.append(obj.fcl_object)
		all_visuals.append(obj)
		
	manager = fcl.DynamicAABBTreeCollisionManager()
	manager.registerObjects(all_objects)  # list of all your CollisionObjects
	manager.setup()  # Builds the BVH tree

	col_res = []

	base_in_world_mat = pose.xyzabc_to_T(base_in_world)
	frame_in_world_inv = pose.inv_dh(pose.xyzabc_to_T(frame_in_world))

	aux_dir_1 = base_in_world_mat @ np.array([aux_dir[0][0], aux_dir[0][1], aux_dir[0][2],0])
	aux_dir_2 = base_in_world_mat @ np.array([aux_dir[1][0], aux_dir[1][1], aux_dir[1][2],0])

	j = joint

	base_mat = np.array(base_in_world_mat)
	aux_offset = np.array([0,0,0])


	if len(j)>6:
		aux_offset = aux_offset + j[6] * aux_dir_1
	if len(j)>7:
		aux_offset = aux_offset + j[7] * aux_dir_2

	base_mat[0, 3] += aux_offset[ 0]
	base_mat[1, 3] += aux_offset[ 1]
	base_mat[2, 3] += aux_offset[ 2]

	robot.set_joint_values([j[0],j[1],j[2],j[3],j[4],j[5]], frame_in_world_inv @  base_mat) 

	#update dynamics
	for do in dynamic_objects:
		manager.update(do.fcl_object)

	cdata = fcl.CollisionData()

	def my_collect_all_callback(o1, o2, cdata):
		fcl.collide(o1, o2, cdata.request, cdata.result)
		return False

	manager.collide(cdata, my_collect_all_callback)#fcl.defaultCollisionCallback)

	tmp_res = None

	for contact in cdata.result.contacts:
		# Extract collision geometries that are in contact
		coll_geom_0 = contact.o1
		coll_geom_1 = contact.o2

		prnt0 = None
		prnt1 = None

		num_parents = 0

		if id(coll_geom_0) in robot.prnt_map:
			prnt0 = robot.prnt_map[id(coll_geom_0)]
			num_parents = num_parents + 1

		if id(coll_geom_1) in robot.prnt_map:
			prnt1 = robot.prnt_map[id(coll_geom_1)]
			num_parents = num_parents + 1


		#this collision has nothing to do with robot
		if num_parents == 0:
			continue 

		#this is external collision, good to go
		if num_parents == 1: 
			pass

		#this is internal collision, needs to be filtered
		if num_parents == 2:
			if prnt0.parent == prnt1 or prnt1.parent == prnt0 or prnt0 == prnt1:
				continue

		#if here, meaning that a valid collision has been detected
		tmp_res = ['scene' if prnt0 is None else prnt0.name, 'scene' if prnt1 is None else prnt1.name]


	if tmp_res is not None:
		col_res.append({"links":tmp_res})

	return col_res


if __name__ == '__main__':
	res = check_collision([0,0,170,0,0,0]) # test function
	print(res)