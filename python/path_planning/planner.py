from . import core
import numpy as np
import fcl
from dorna2 import pose, Dorna
from importlib.resources import files, as_file

from . import urdf
from . import node

def mm_to_m_6(xyzabc):
	return (np.array(xyzabc)/1000).tolist()

def m_to_mm_6(xyzabc):
	return (np.array(xyzabc)*1000).tolist()

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
		link_boxes=None,			 # {link_name: [cube, ...]} — boxes that BELONG to a robot link
		dorna=None
	):
		self.tool = [0, 0, 0, 0, 0, 0] if tool is None else tool
		self.gripper = [] if gripper is None else gripper
		self.load = [] if load is None else load
		self.scene = [] if scene is None else scene
		self.base_in_world = [0, 0, 0, 0, 0, 0] if base_in_world is None else base_in_world
		self.frame_in_world = [0, 0, 0, 0, 0, 0] if frame_in_world is None else frame_in_world
		self.aux_dir = [[0, 0, 0], [0, 0, 0]] if aux_dir is None else [[aux_dir[0][0]/1000, aux_dir[0][1]/1000, aux_dir[0][2]/1000],
				[aux_dir[1][0]/1000, aux_dir[1][1]/1000, aux_dir[1][2]/1000]]
		self.aux_limit = [[-1, 1], [-1, 1]] if aux_limit is None else aux_limit
		# Boxes attached to a robot LINK (a camera bolted to the wrist, a
		# bracket on the forearm): {link_name: [cube, ...]}, each cube's pose
		# in that link's frame. They move with the link and collide with the
		# world and with non-adjacent links exactly as the link's own
		# geometry does — same link / parent-child pairs are never a hit.
		# Gripper boxes are the j6_link case of the same idea.
		self.link_boxes = {} if link_boxes is None else dict(link_boxes)
		self.dorna = Dorna() if dorna is None else dorna
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
		gripper=None,
		link_boxes=None,
		dorna=None
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
			self.aux_dir = [[self.aux_dir[0][0]/1000, self.aux_dir[0][1]/1000, self.aux_dir[0][2]/1000],
				[self.aux_dir[1][0]/1000, self.aux_dir[1][1]/1000, self.aux_dir[1][2]/1000]]
		if aux_limit is not None:
			self.aux_limit = aux_limit
		if gripper is not None:
			self.gripper = gripper
		if link_boxes is not None:
			self.link_boxes = dict(link_boxes)
		if dorna is not None:
			self.dorna = dorna

		self.rebuild()

	def rebuild(self):

		#limits
		if hasattr(self.dorna.kinematic, 'limits'):
			limits = self.dorna.kinematic.limits
		else:
			limits = {"j0":[-180,180],"j1":[-180,180],"j2":[-180,180],"j3":[-180,180],"j4":[-180,180],"j5":[-180,180]}
			
		self.limit_n       = [limits["j0"][0],limits["j1"][0],limits["j2"][0],limits["j3"][0],limits["j4"][0],limits["j5"][0],self.aux_limit[0][0], self.aux_limit[1][0]]
		self.limit_p       = [limits["j0"][1],limits["j1"][1],limits["j2"][1],limits["j3"][1],limits["j4"][1],limits["j5"][1],self.aux_limit[0][1], self.aux_limit[1][1]]

		#mm to m — into their own names: the stored parameters stay in mm as
		#given, so an update() of a subset never rescales the rest
		self.tool_m = mm_to_m_6(self.tool)
		self.base_in_world_m = mm_to_m_6(self.base_in_world)
		self.frame_in_world_m = mm_to_m_6(self.frame_in_world)


		#rebuilding initialization stuff
		self.root_node = node.Node("root")

		urdf_path = res = files("path_planning") / "resources" / "urdf" / "dorna_ta.urdf"

		self.robot = urdf.urdf_robot(urdf_path, {}, self.root_node)

		self.all_visuals = [] #for visualization
		self.all_objects = [] #to create bvh
		self.dynamic_objects = [] #to update bvh

		self.scene_map = {}
		#placing scene objects
		for obj in self.scene:
			self.root_node.collisions.append(obj)
			self.all_objects.append(obj.fcl_object)
			self.all_visuals.append(obj)
			self.scene_map[id(obj.fcl_shape)] = obj


		#placing tool objects
		for obj in self.load:
			#create new obj
			new_pose = m_to_mm_6(pose.T_to_xyzabc(np.matrix(pose.xyzabc_to_T(self.tool_m)) @ np.matrix(pose.xyzabc_to_T(mm_to_m_6(obj.pose)))))
			new_obj = Planner.create_cube(new_pose, [obj.scale[0], obj.scale[1], obj.scale[2]])

			self.robot.link_nodes["j6_link"].collisions.append(new_obj)
			self.robot.all_objs.append(new_obj)
			self.robot.prnt_map[id(new_obj.fcl_shape)] = self.robot.link_nodes["j6_link"]

		#placing tool objects
		for obj in self.gripper:
			self.robot.link_nodes["j6_link"].collisions.append(obj)
			self.robot.all_objs.append(obj)
			self.robot.prnt_map[id(obj.fcl_shape)] = self.robot.link_nodes["j6_link"]

		#placing link-mounted objects: part of their link, like the gripper is of j6_link
		for link_name, objs in self.link_boxes.items():
			if link_name not in self.robot.link_nodes:
				raise ValueError("link_boxes: no link %r (links: %s)" % (link_name, ", ".join(self.robot.link_nodes)))
			nod = self.robot.link_nodes[link_name]
			for obj in objs:
				nod.collisions.append(obj)
				self.robot.all_objs.append(obj)
				self.robot.prnt_map[id(obj.fcl_shape)] = nod

		#registering robot objects
		for obj in self.robot.all_objs:
			self.dynamic_objects.append(obj)
			self.all_objects.append(obj.fcl_object)
			self.all_visuals.append(obj)
			
		self.manager = fcl.DynamicAABBTreeCollisionManager()
		self.manager.registerObjects(self.all_objects)  # list of all your CollisionObjects
		self.manager.setup()  # Builds the BVH tree

		self.base_in_world_mat = pose.xyzabc_to_T(self.base_in_world_m)
		self.frame_in_world_inv = np.linalg.inv(pose.xyzabc_to_T(self.frame_in_world_m))

		self.aux_dir_1 = self.base_in_world_mat @ np.array([self.aux_dir[0][0], self.aux_dir[0][1], self.aux_dir[0][2], 0])
		self.aux_dir_2 = self.base_in_world_mat @ np.array([self.aux_dir[1][0], self.aux_dir[1][1], self.aux_dir[1][2], 0])


	def link_frames(self, joint, base_in_world=None):
		"""Where the robot's links are at ``joint`` (degrees; rail axes in mm
		after index 5): ``{link_name: 4x4 world transform in METRES}`` for
		every URDF link — the planner's own FK, the frame a link box's pose
		is relative to. The scene tree is a second model of the same robot
		with its own link frames; a box that belongs to a link is expressed
		here from its world pose, never by assuming the two frames coincide.
		``base_in_world`` (mm) overrides the stored base for this call, so
		the frames can be asked for BEFORE the update that sets it."""
		j = list(joint)
		if base_in_world is None:
			base_mat = np.array(self.base_in_world_mat)
			aux_1, aux_2 = self.aux_dir_1, self.aux_dir_2
		else:
			base_mat = np.array(pose.xyzabc_to_T(mm_to_m_6(base_in_world)))
			aux_1 = base_mat @ np.array([self.aux_dir[0][0], self.aux_dir[0][1], self.aux_dir[0][2], 0])
			aux_2 = base_mat @ np.array([self.aux_dir[1][0], self.aux_dir[1][1], self.aux_dir[1][2], 0])
		aux_offset = np.array([0, 0, 0])
		if len(j) > 6:
			aux_offset = aux_offset + j[6] * aux_1[:3]
		if len(j) > 7:
			aux_offset = aux_offset + j[7] * aux_2[:3]
		base_mat[0, 3] += aux_offset[0]
		base_mat[1, 3] += aux_offset[1]
		base_mat[2, 3] += aux_offset[2]
		root = self.frame_in_world_inv @ base_mat
		self.robot.set_joint_values([j[0], j[1], j[2], j[3], j[4], j[5]], root)
		# Node.get_global_transform() is the chain below the root; the root
		# (base on the rail, in the planner frame) is applied on top.
		frame = np.array(pose.xyzabc_to_T(self.frame_in_world_m))
		return {name: frame @ root @ np.array(nod.get_global_transform(), dtype=float)
				for name, nod in self.robot.link_nodes.items()}

	def robot_boxes(self, joint, base_in_world=None):
		"""The robot as THIS planner sees it at ``joint``: every URDF link's
		collision boxes (and any gripper / link boxes attached), each as
		``{"link", "pose": [x, y, z, a, b, c] in WORLD mm, "scale": mm}``.
		For drawing the planner's robot next to the scene's — what collision
		is checked against, where the planner believes it stands."""
		self.link_frames(joint, base_in_world)           # poses every collision object (gt)
		frame = np.array(pose.xyzabc_to_T(self.frame_in_world_m))
		out = []
		for name, nod in self.robot.link_nodes.items():
			for obj in nod.collisions:
				T = frame @ np.array(obj.gt, dtype=float)
				T = T.copy(); T[:3, 3] *= 1000.0
				out.append({"link": name, "pose": [float(v) for v in pose.T_to_xyzabc(T)],
							"scale": [float(v) * 1000.0 for v in obj.scale]})
		return out

	def check_collision(self, joint, internal=True):

		#check limits
		for i in range(len(joint)):
			if joint[i]<self.limit_n[i] or joint[i]>self.limit_p[i]:
				return [{"links":["j"+str(i)+"_limit",None]}]			


		col_res = []

		j = joint

		base_mat = np.array(self.base_in_world_mat)
		aux_offset = np.array([0,0,0])


		if len(j)>6:
			aux_offset = aux_offset + j[6] * self.aux_dir_1[:3]

		if len(j)>7:
			aux_offset = aux_offset + j[7] * self.aux_dir_2[:3]

		base_mat[0, 3] += aux_offset[ 0]
		base_mat[1, 3] += aux_offset[ 1]
		base_mat[2, 3] += aux_offset[ 2]



		self.robot.set_joint_values([j[0],j[1],j[2],j[3],j[4],j[5]], self.frame_in_world_inv @  base_mat) 

		"""	
		st = ""
		for o in self.all_visuals:
			pp = pose.T_to_xyzabc(o.gt)
			if pp[0] == 0.0 and pp[1] == 0.0 and pp[2] ==0.0:
				pp = pose.T_to_xyzabc(o.mat)

			st += '{"pose":' + str(pp) + ',"scale":'+str(o.scale) + "},"
		
		print(st)
		"""
		
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

			#this is external collision, good to go if internal = false
			if num_parents == 1:
				if  internal:
					continue

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


	def plan(self, start, goal, seed=1234, gravity=False, gravity_vec=[0,0,1], gravity_thr = 1.0, planner="rrtconnect", time_limit_sec=2.0, rail_weight=0.01, joint_weights=None):
		# Degenerate query — start and goal are the same state. Nothing to
		# plan, and the informed planners (AIT*/BIT*) would throw from
		# OMPL's ProlateHyperspheroid (zero-width sampling ellipsoid).
		if max(abs(float(s) - float(g)) for s, g in zip(start, goal)) < 1e-6:
			return [list(map(float, start)), list(map(float, goal))]

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

		link_list = []
		for link_name, objs in self.link_boxes.items():
			for obj in objs:
				link_list.append({
								"link":  link_name,
								"pose":  obj.pose,          # vec6, in the link's frame
								"scale": obj.scale,                # vec3
								"type":  core.ShapeType.Box         # enum
								})

		dof = len(start)
		
		if not gravity :
			gravity_vec = [0,0,1]

		plan_args = dict(
						start_joint   = np.array(start, dtype=float),
						goal_joint    = np.array(goal, dtype=float),
						limit_n       = np.array(self.limit_n[:dof], dtype=float),
						limit_p       = np.array(self.limit_p[:dof], dtype=float),
						scene         = scene_list,
						load          = load_list,
						gripper       = gripper_list,
						tool          = np.array(self.tool_m, dtype=float),
						base_in_world = np.array(self.base_in_world_m, dtype=float),
						frame_in_world= np.array(self.frame_in_world_m, dtype=float),
						aux_dir       = self.aux_dir,
						time_limit_sec= time_limit_sec,
						seed 		  = seed,
						link_boxes    = link_list,
						gravity		  = gravity,
						gravity_vec	  = np.array(gravity_vec, dtype=float).reshape(3, 1),
						gravity_thr	  = gravity_thr,
						rail_weight	  = rail_weight,
						joint_weights = list(joint_weights) if joint_weights else [],
						)

		try:
			path = core.plan(planner=planner, **plan_args)
		except RuntimeError as e:
			# OMPL informed-sampler degeneracy: when the straight-line
			# connection IS the optimum, the ProlateHyperspheroid used by
			# AIT*/BIT* collapses and throws mid-solve. RRTConnect never
			# samples informed — same collision checker, same constraints.
			if "PHS" not in str(e) and "transverse diameter" not in str(e):
				raise
			path = core.plan(planner="rrtconnect", **plan_args)
		return path


	def check(self, path, gravity=False, gravity_vec=[0,0,1], gravity_thr = 1.0, rail_weight=0.01, joint_weights=None):
		"""Revalidate a stored path (degrees, list of joint lists) against
		the CURRENT scene/load/gripper — same validity checker the
		planners use. True only if every segment is valid."""
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

		link_list = []
		for link_name, objs in self.link_boxes.items():
			for obj in objs:
				link_list.append({"link": link_name, "pose": obj.pose, "scale": obj.scale, "type": core.ShapeType.Box})
		dof = len(path[0])

		if not gravity :
			gravity_vec = [0,0,1]

		return core.check_path(
						path          = [[float(v) for v in p] for p in path],
						limit_n       = np.array(self.limit_n[:dof], dtype=float),
						limit_p       = np.array(self.limit_p[:dof], dtype=float),
						scene         = scene_list,
						load          = load_list,
						gripper       = gripper_list,
						tool          = np.array(self.tool_m, dtype=float),
						base_in_world = np.array(self.base_in_world_m, dtype=float),
						frame_in_world= np.array(self.frame_in_world_m, dtype=float),
						aux_dir       = self.aux_dir,
						link_boxes    = link_list,
						gravity       = gravity,
						gravity_vec   = np.array(gravity_vec, dtype=float).reshape(3, 1),
						gravity_thr   = gravity_thr,
						rail_weight   = rail_weight,
						joint_weights = list(joint_weights) if joint_weights else [],
						)
