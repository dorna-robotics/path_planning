import dorna2.pose as dorna_pose
import numpy as np
from . import planner

class Inverse_kinematic_error(Exception):
    """Raised when inverse kinematics cannot find a valid solution."""
    pass


class Kinematic():
    def __init__(self, robot=None, base_in_world=[0, 0, 0, 0, 0, 0]):
        self.robot = robot
        self.base_in_world = base_in_world
        self.planner = planner.Planner(base_in_world=self.base_in_world)


    def inv(self, pose_in_world, aux=[0, 0], tool=[0, 0, 0, 0, 0, 0], init_joint=None, freedom=None):
        
        self.planner.update(tool=tool)
        joint = []

        try:
            # current joint
            self.robot.kinematic.set_tcp_xyzabc(tool)
            current_joint = init_joint if init_joint is not None else self.robot.get_all_joint()
            current_joint = (current_joint + [0, 0, 0, 0, 0, 0, 0, 0])[:8]

            # target joint
            pose_in_robot = np.array(dorna_pose.frame_to_robot(pose_in_world,
                                                            aux=aux, 
                                                            base_in_world=self.base_in_world)) 
            _joint = np.append(self.robot.kinematic.inv(pose_in_robot, current_joint[0:6], False, freedom=freedom)[0], aux)
            
            # check if there is a collision then set the joint
            for j in _joint.tolist():

                #set aux values
                j = j[:6] + aux

                res = self.planner.check_collision(joint=j)
                if len(res)>0: #some collision has happens
                    continue
                else:
                    joint.append(j)

            #joint = _joint.tolist()
                       
        except Inverse_kinematic_error:
            # re-raise so main loop can catch it
            raise
        finally:
            # reset tool
            self.robot.kinematic.set_tcp_xyzabc([0, 0, 0, 0, 0, 0])


        return joint, {"j"+str(i): float(joint[i]) for i in range(len(joint))}