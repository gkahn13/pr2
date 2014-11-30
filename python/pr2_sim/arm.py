import numpy as np

import roslib
roslib.load_manifest('tfx')
import tfx
import tf.transformations as tft

import openravepy as rave

import simulator
from utils import utils

import IPython

class Arm:
    """
    Class for controlling the end effectors of the PR2 in OpenRAVE
    """
    
    joint_name_suffixes = ["_shoulder_pan_joint",
                           "_shoulder_lift_joint",
                           "_upper_arm_roll_joint",
                           "_elbow_flex_joint",
                           "_forearm_roll_joint",
                           "_wrist_flex_joint",
                           "_wrist_roll_joint"]
    
    L_POSTURES = {
                  'untucked' : [0.4,  1.0,    0.0,  -2.05,  0.0,  -0.1,   0.0],
                  'tucked'   : [0.06, 1.25,   1.79, -1.68, -1.73, -0.10, -0.09],
                  'up'       : [0.33, -0.35,  2.59, -0.15,  0.59, -1.41, -0.27],
                  'side'     : [1.83, -0.33,  1.01, -1.43,  1.1,  -2.10,  3.07],
                  'mantis'   : [2.03, -0.054, 1.01, -1.47,  0.55, -1.42,  0.82]
                  }
    
    
    def __init__(self, arm_name, sim=None, view=True):
        """
        :param arm_name: "left" or "right"
        :param sim: OpenRave simulator (or create if None)
        """
        assert arm_name == 'left' or arm_name == 'right'
        
        self.arm_name = arm_name
        self.tool_frame = '{0}_gripper_tool_frame'.format(arm_name[0])
        self.joint_names = ['{0}{1}'.format(arm_name[0], joint_name) for joint_name in self.joint_name_suffixes]
        
        self.sim = sim
        if self.sim is None:
            self.sim = simulator.Simulator()
              
            if view:
                self.sim.env.SetViewer('qtcoin')
            
        self.robot = self.sim.robot
        self.manip = self.sim.larm if arm_name == 'left' else self.sim.rarm   
        self.tool_frame_link = self.robot.GetLink(self.tool_frame)
        
        self.gripper_joint = self.robot.GetJoint('{0}_gripper_l_finger_joint'.format(arm_name[0]))
        
        self.joint_indices = self.manip.GetArmIndices()

    #####################
    # command methods   #
    #####################
    
    def set_posture(self, posture):
        """
        :param posture: 'untucked', 'tucked', 'up', 'side', 'mantis'
        """
        assert posture in self.L_POSTURES.keys()
        
        l_joints = self.L_POSTURES[posture]
        joints = l_joints if self.arm_name == "left" else Arm._mirror_arm_joints(l_joints)
        self.set_joints(joints)
    
    def set_pose(self, pose):
        """
        :param pose: tfx.pose
        """
        joints = self.ik(pose)
        if joints is not None:
            self.set_joints(joints)
    
    def set_joints(self, joint_values):
        """
        :param joints: list of joint values
        """
        l_limits, u_limits = self.get_joint_limits()
        valid_joint_values = np.array(joint_values)
        valid_joint_values = np.maximum(joint_values, l_limits)
        valid_joint_values = np.minimum(joint_values, u_limits)
        if not np.min(np.array(joint_values) == valid_joint_values):
            print('Invalid joints. Setting invalid values to limits')
        self.robot.SetDOFValues(joint_values, self.joint_indices)
        
    def teleop(self):
        print('{0} arm teleop'.format(self.arm_name))
        
        pos_step = .01
        delta_position = {'a' : [0, pos_step, 0],
                          'd' : [0, -pos_step, 0],
                          'w' : [pos_step, 0, 0],
                          'x' : [-pos_step, 0, 0],
                          '+' : [0, 0, pos_step],
                          '-' : [0, 0, -pos_step]}
        
        angle_step = 2.0
        delta_angle = {'o' : [angle_step, 0, 0],
                       'p' : [-angle_step, 0, 0],
                       'k' : [0, angle_step, 0],
                       'l' : [0, -angle_step, 0],
                       'n' : [0, 0, angle_step],
                       'm' : [0, 0, -angle_step]}
        
        char = ''
        while char != 'q':
            char = utils.Getch.getch()
            pose = self.get_pose()
            new_pose = tfx.pose(pose)
            if delta_position.has_key(char):
                new_pose.position = pose.position.array + delta_position[char]
            ypr = np.array([pose.tb_angles.yaw_deg, pose.tb_angles.pitch_deg, pose.tb_angles.roll_deg])
            if delta_angle.has_key(char):
                ypr += np.array(delta_angle[char])
                new_pose = tfx.pose(pose.position, tfx.tb_angles(ypr[0], ypr[1], ypr[2]))
            new_joints = self.ik(new_pose)
            if new_joints is not None:
                self.set_joints(new_joints)
            else:
                print('teleop: IK invalid')
            
        print('{0} arm end teleop'.format(self.arm_name))
        
    def open_gripper(self):
        self.set_gripper(1.0)
    
    def close_gripper(self):
        self.set_gripper(0.0)
    
    def set_gripper(self, pct):
        """
        Set gripper to percentage open
        
        :param pct: [0,1] amount the gripper is open
        """
        lower, upper = self.gripper_joint.GetLimits()
        value = pct*(upper[0] - lower[0]) + lower[0]
        index = self.gripper_joint.GetDOFIndex()
        self.robot.SetDOFValues([value], [index])
        
    
    #######################
    # state info methods  #
    #######################
    
    def get_pose(self):
        """
        :return current pose of tool_frame as tfx.pose
        """
        return tfx.pose(self.sim.transform_from_to(np.eye(4), self.tool_frame, 'base_link'), frame='base_link')
    
    def get_joints(self):
        """
        :return current joint values
        """
        return self.manip.GetArmDOFValues()
    
    def get_joint_limits(self):
        """
        :return [lower limits array, upper limits array]
        """
        l_limits, u_limits = self.robot.GetDOFLimits()
        arm_l_limits = np.array([l_limits[i] for i in self.joint_indices])
        arm_u_limits = np.array([u_limits[i] for i in self.joint_indices])
        return arm_l_limits, arm_u_limits
    
    
    #############
    #   fk/ik   #
    #############
    
    def fk(self, joints):
        """
        :return tfx.pose of tool_frame pose
        """
        curr_joints = self.get_joints()
        
        self.set_joints(joints)
        pose = self.get_pose()
    
        self.set_joints(curr_joints)
        return pose
    
    def ik(self, pose):
        """
        :param pose: tfx.pose
        :return list of joints or None if no solution
        """
        assert pose.frame == 'base_link'
        
        pose_mat_world = self.sim.transform_from_to(pose.matrix, pose.frame, 'world')
        joints = self.sim.ik_for_link(pose_mat_world, self.manip, self.tool_frame, 0)
        
        if joints is not None:
            joints = self._closer_joint_angles(joints, self.get_joints())
            
        return joints
    
    def ik_lookat(self, position, point):
        """
        :param position: desired position of tool_frame (tfx.point)
        :param point: desired point for tool_frame to point at (tfx.point)
        """
        assert position.frame == 'base_link'
        
        position_world = self.sim.transform_from_to(position.array, 'base_link', 'world')
        direction_world = np.dot(self.sim.transform_from_to(np.eye(4), 'base_link', 'world')[:3,:3], point.array - position.array)
        direction_world /= np.linalg.norm(direction_world)
        
        red = direction_world
        red /= np.linalg.norm(red)
        green = np.cross(red, np.array([0,0,1]))
        green /= np.linalg.norm(red)
        blue = np.cross(red, green)
        blue /= np.linalg.norm(blue)
        
        pose_mat = np.eye(4)
        pose_mat[:3,0] = red
        pose_mat[:3,1] = green
        pose_mat[:3,2] = blue
        pose_mat[:3,3] = position_world
        
        pose = tfx.pose(self.sim.transform_from_to(pose_mat, 'world', 'base_link'), frame='base_link')
        return self.ik(pose)
        
    ##################
    # helper methods #
    ##################
    
    @staticmethod
    def _mirror_arm_joints(x):
        """"
        Mirror image of joints (r->l or l->r)
        
        :param x: joints
        """
        return np.array([-x[0],x[1],-x[2],x[3],-x[4],x[5],-x[6]])
    
    @staticmethod
    def _closer_joint_angles(new_joints, curr_joints):
        for i in [2, 4, 6]:
            new_joints[i] = utils.closer_angle(new_joints[i], curr_joints[i])
        return new_joints
    
#############
#   TESTS   #
#############
    
def test_ik():
    arm = Arm('right')
    arm.set_posture('mantis')
    
    actual_joints = arm.get_joints()
    actual_pose = arm.get_pose()
    
    ik_joints = arm.ik(actual_pose)
    
    print('actual_joints: {0}'.format(actual_joints))
    print('ik_joints: {0}'.format(ik_joints))

def test_teleop():
    arm = Arm('right')
    arm.set_posture('mantis')
    
    arm.teleop()
    
    
if __name__ == '__main__':
    test_ik()
    #test_teleop()