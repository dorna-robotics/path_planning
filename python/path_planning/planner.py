from . import core
import numpy as np
import fcl
from dorna2 import pose, Dorna
from importlib.resources import files, as_file

from . import urdf
from . import node

def mm_to_m_6(xyzabc):
	xyzabc[0] = xyzabc[0]/1000
	xyzabc[1] = xyzabc[1]/1000
	xyzabc[2] = xyzabc[2]/1000
	return xyzabc

def m_to_mm_6(xyzabc):
	xyzabc[0] = xyzabc[0]*1000
	xyzabc[1] = xyzabc[1]*1000
	xyzabc[2] = xyzabc[2]*1000
	return xyzabc

class Planner:

	def __init__(
		self,
		*,
		tool=None,					  # [x,y,z,rx,ry,rz]
		load=None,					  # list of tools attached
		gripper=None,
		scene=None,					 # list of obstacles in scene
		base_in_world=None,			 # [x,y,z,rx,ry,rz]
		frame_in_world=None,			# [x,y,z,rx,ry,rz]
		aux_dir=None,				   # [[...],[...]]
		aux_limit=None,				 # [[min,max],[min,max]]
	):
		self.tool = [0, 0, 0, 0, 0, 0] if tool is None else tool
		self.gripper = [] if gripper is None else gripper
		self.load = [] if load is None else load
		self.scene = [] if scene is None else scene
		self.base_in_world = [0, 0, 0, 0, 0, 0] if base_in_world is None else base_in_world
		self.frame_in_world = [0, 0, 0, 0, 0, 0] if frame_in_world is None else frame_in_world
		self.aux_dir = [[0, 0, 0], [0, 0, 0]] if aux_dir is None else aux_dir
		self.aux_limit = [[-1, 1], [-1, 1]] if aux_limit is None else aux_limit

		self.rebuild()

	def update(
		self,
		*,
		tool=None,
		load=None,
		scene=None,
		base_in_world=None,
		frame_in_world=None,
		aux_dir=None,
		aux_limit=None,
		gripper=None
	):
		"""Update any subset of stored parameters."""
		if tool is not None:
			self.tool = tool
		if load is not None:
			self.load = load
		if scene is not None:
			self.scene = scene
		if base_in_world is not None:
			self.base_in_world = base_in_world
		if frame_in_world is not None:
			self.frame_in_world = frame_in_world
		if aux_dir is not None:
			self.aux_dir = aux_dir
		if aux_limit is not None:
			self.aux_limit = aux_limit
		if gripper is not None:
			self.gripper = gripper
		self.rebuild()

	def rebuild(self):

		#limits
		self.dorna = Dorna()
		limits = self.dorna.kinematic.limits
		self.limit_n       = [limits["j0"][0],limits["j1"][0],limits["j2"][0],limits["j3"][0],limits["j4"][0],limits["j5"][0],self.aux_limit[0][0], self.aux_limit[1][0]]
		self.limit_p       = [limits["j0"][1],limits["j1"][1],limits["j2"][1],limits["j3"][1],limits["j4"][1],limits["j5"][1],self.aux_limit[0][1], self.aux_limit[1][1]]
		
		#mm to m
		self.tool = mm_to_m_6(self.tool)
		self.base_in_world = mm_to_m_6(self.base_in_world)
		self.frame_in_world = mm_to_m_6(self.frame_in_world )
		self.aux_dir = [[self.aux_dir[0][0]/1000, self.aux_dir[0][1]/1000, self.aux_dir[0][2]/1000],
						[self.aux_dir[1][0]/1000, self.aux_dir[1][1]/1000, self.aux_dir[1][2]/1000]]

		#rebuilding initialization stuff
		self.root_node = node.Node("root")

		urdf_path = res = files("path_planning") / "resources" / "urdf" / "dorna_ta.urdf"

		self.robot = urdf.urdf_robot(urdf_path, {}, self.root_node)

		self.all_visuals = [] #for visualization
		self.all_objects = [] #to create bvh
		self.dynamic_objects = [] #to update bvh

		#placing scene objects
		for obj in self.scene:
			self.root_node.collisions.append(obj.fcl_object)
			self.all_objects.append(obj.fcl_object)
			self.all_visuals.append(obj)

		#placing tool objects
		for obj in self.load:
			#create new obj
			new_pose = m_to_mm_6(pose.T_to_xyzabc(np.matrix(pose.xyzabc_to_T(self.tool)) @ np.matrix(pose.xyzabc_to_T(mm_to_m_6(obj.pose)))))
			new_obj = Planner.create_cube(new_pose, [obj.scale[0]*1000, obj.scale[1]*1000, obj.scale[2]*1000])

			self.robot.link_nodes["j6_link"].collisions.append(new_obj)
			self.robot.all_objs.append(new_obj)
			self.robot.prnt_map[id(new_obj.fcl_shape)] = self.robot.link_nodes["j6_link"]

		#placing tool objects
		for obj in self.gripper:
			self.robot.link_nodes["j6_link"].collisions.append(obj)
			self.robot.all_objs.append(obj)
			self.robot.prnt_map[id(obj.fcl_shape)] = self.robot.link_nodes["j6_link"]

		#registering robot objects
		for obj in self.robot.all_objs:
			self.dynamic_objects.append(obj)
			self.all_objects.append(obj.fcl_object)
			self.all_visuals.append(obj)
			
		self.manager = fcl.DynamicAABBTreeCollisionManager()
		self.manager.registerObjects(self.all_objects)  # list of all your CollisionObjects
		self.manager.setup()  # Builds the BVH tree

		self.base_in_world_mat = pose.xyzabc_to_T(self.base_in_world)
		self.frame_in_world_inv = np.linalg.inv(pose.xyzabc_to_T(self.frame_in_world))

		self.aux_dir_1 = self.base_in_world_mat @ np.array([self.aux_dir[0][0], self.aux_dir[0][1], self.aux_dir[0][2], 0])
		self.aux_dir_2 = self.base_in_world_mat @ np.array([self.aux_dir[1][0], self.aux_dir[1][1], self.aux_dir[1][2], 0])


	def check_collision(self, joint):

		#check aux limits
		if len(joint)>6:
			if joint[6]<self.aux_limit[0][0] or joint[6]>self.aux_limit[0][1]:
				return [{"links":["aux0_limit",None]}]
		if len(joint)>7:
			if joint[7]<self.aux_limit[1][0] or joint[7]>self.aux_limit[1][1]:
				return [{"links":["aux1_limit",None]}]

		col_res = []

		j = joint

		base_mat = np.array(self.base_in_world_mat)
		aux_offset = np.array([0,0,0])


		if len(j)>6:
			aux_offset = aux_offset + j[6] * self.aux_dir_1
		if len(j)>7:
			aux_offset = aux_offset + j[7] * self.aux_dir_2

		base_mat[0, 3] += aux_offset[ 0]
		base_mat[1, 3] += aux_offset[ 1]
		base_mat[2, 3] += aux_offset[ 2]

		self.robot.set_joint_values([j[0],j[1],j[2],j[3],j[4],j[5]], self.frame_in_world_inv @  base_mat) 

		#update dynamics
		for do in self.dynamic_objects:
			self.manager.update(do.fcl_object)

		cdata = fcl.CollisionData()

		def my_collect_all_callback(o1, o2, cdata):
			fcl.collide(o1, o2, cdata.request, cdata.result)
			return False

		self.manager.collide(cdata, my_collect_all_callback)#fcl.defaultCollisionCallback)

		tmp_res = None

		for contact in cdata.result.contacts:
			# Extract collision geometries that are in contact
			coll_geom_0 = contact.o1
			coll_geom_1 = contact.o2

			prnt0 = None
			prnt1 = None

			num_parents = 0

			if id(coll_geom_0) in self.robot.prnt_map:
				prnt0 = self.robot.prnt_map[id(coll_geom_0)]
				num_parents = num_parents + 1

			if id(coll_geom_1) in self.robot.prnt_map:
				prnt1 = self.robot.prnt_map[id(coll_geom_1)]
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


	def create_cube(pose, scale):
		return node.create_cube(xyz_rvec=pose, scale=scale)


	def plan(self, start, goal):
		scene_list = []
		gripper_list = []
		load_list = []

		for obj in self.scene:
			scene_list.append({
								"pose":  obj.pose,          # vec6
								"scale": obj.scale,                # vec3
								"type":  core.ShapeType.Box         # enum 
								})

		for obj in self.load:
			load_list.append({
								"pose":  obj.pose,          # vec6
								"scale": obj.scale,                # vec3
								"type":  core.ShapeType.Box         # enum 
								})		

		for obj in self.gripper:
			gripper_list.append({
								"pose":  obj.pose,          # vec6
								"scale": obj.scale,                # vec3
								"type":  core.ShapeType.Box         # enum 
								})	

		dof = len(start)

		path = core.plan(
						start_joint   = np.array(start, dtype=float),
						goal_joint    = np.array(goal, dtype=float),
						limit_n       = np.array(self.limit_n[:dof], dtype=float),
						limit_p       = np.array(self.limit_p[:dof], dtype=float),
						scene         = scene_list,
						load          = load_list,
						gripper       = gripper_list,
						tool          = np.array(self.tool, dtype=float),
						base_in_world = np.array(self.base_in_world, dtype=float),
						frame_in_world= np.array(self.frame_in_world, dtype=float),
						aux_dir       = self.aux_dir,
						time_limit_sec= 2.0
						)
		return path