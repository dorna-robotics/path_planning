from urchin import URDF
import numpy as np
import fcl
import time

import dorna2.pose as dp
from . import node

class urdf_robot:

    def urdf_joint_transform(self, joint, value=0.0):
        """Get joint transform at a given joint value (for revolute or prismatic)."""
        T = np.eye(4)
        T = joint.origin

        axis = np.array(joint.axis)
        if joint.joint_type == 'revolute':
            R_axis = dp.abc_to_rmat(value * axis)
            motion = np.eye(4)
            motion[:3, :3] = R_axis
            return T @ motion

        elif joint.joint_type == 'prismatic':
            motion = np.eye(4)
            motion[:3, 3] = value * axis
            return T @ motion

        else:  # fixed or unsupported
            return T

    def __init__(self, urdf_file, joint_values={}, parent = None):
        self.robot = URDF.load(urdf_file)
        self.all_objs = []
        self.link_nodes = {}
        self.link_names = []
        self.prnt_map = {}

        def recurse(link_name, parent_node):
            link = self.robot.link_map[link_name]
            self.link_names.append(link_name)
            joint = next((j for j in self.robot.joints if j.child == link_name), None)
            joint_value = joint_values.get(joint.name, 0.0) if joint else 0.0

            local_tf = self.urdf_joint_transform(joint, joint_value) if joint else np.eye(4)

            nod = node.Node(link.name, parent_node, local_tf)
            nod.lock = True

            self.link_nodes[link.name] = nod


            # Visuals and Collisions
            #for vis in link.visuals:
            #    node.visuals.append(vis.geometry)  # optionally convert to your own mesh format

            for col in link.collisions:
                shape = col.geometry
                tf = np.eye(4)            
                if col.origin is not None:
                    tf = np.matrix(col.origin)

                xyzabc = dp.T_to_xyzabc(tf)
                # Example: only support box/sphere/cylinder for now
                if shape.box:
                    half_extents = shape.box.size
                    obj = node.create_cube(xyzabc, scale = half_extents)
                elif shape.sphere:
                    obj = node.create_sphere(xyzabc, scale = [1,1,1])

                else:
                    obj = None
                    continue

                self.prnt_map[id(obj.fcl_shape)] = nod
                nod.collisions.append(obj)
                self.all_objs.append(obj)

            for j in self.robot.joints:
                if j.parent == link_name:
                    recurse(j.child, nod)

        self.root_link = self.robot.base_link
        self.root_node = node.Node(self.root_link.name, parent)
        self.link_nodes[self.root_link.name] = self.root_node
        recurse(self.root_link.name, self.root_node)


    def set_joint_values(self, joints = [0,0,0,0,0,0,0], offset_mat = np.eye(4)):

        def recurse(link_name, parent_node, joints, count ,parent_tf):
            link = self.robot.link_map[link_name]
            joint = next((j for j in self.robot.joints if j.child == link_name), None)
            
            if count>-1:
                joint_value = joints[count]# * np.pi / 180.0
            else:
                joint_value = 0


            local_tf = self.urdf_joint_transform(joint, joint_value) if joint else np.eye(4)


            g_tf = parent_tf @ local_tf

            nod = parent_node.children[0]
            nod.lock = True

            nod.set_local_transform(local_tf)
            nod.update_collision_object_transforms(g_tf)

            for j in self.robot.joints:
                if j.parent == link_name:
                    recurse(j.child, nod, joints, count + 1, g_tf)
        recurse(self.root_link.name, self.root_node, joints, -1 , offset_mat)
