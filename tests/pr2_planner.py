import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np

from pr2 import planner
from pr2_sim import arm, simulator

import IPython

def test_grasp_planner():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    p = planner.Planner('right', sim=sim)
    
    rarm.set_posture('mantis')
    curr_pose = rarm.get_pose()
    curr_pose.orientation = tfx.tb_angles(0,0,180)
    rarm.set_pose(curr_pose)
    
    target_pose = rarm.get_pose() + [0.5, 0.4, -0.3]
    target_pose.orientation *= tfx.tb_angles(25,-20,10)
    sim.plot_transform(target_pose.matrix)

    start_joints = rarm.get_joints()
    n_steps = 20
    joint_traj = p.get_grasp_joint_trajectory(start_joints, target_pose, n_steps=n_steps, ignore_orientation=True, link_name='r_gripper_center_frame')
    
    poses = [tfx.pose(rarm.fk(joints)) for joints in joint_traj]
    for pose in poses:
        sim.plot_transform(pose.matrix)
        
    for n in xrange(n_steps-1):
        angle = (poses[n].orientation.inverse()*poses[-1].orientation).angle
        print('angle[{0}]: {1}'.format(n, angle))
    
    #IPython.embed()
    
    print('Press enter to exit')
    raw_input()
    
def test_return_from_grasp_planner():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    p = planner.Planner('right', sim=sim)
    
    rarm.set_posture('mantis')
    curr_pose = rarm.get_pose()
    curr_pose.orientation = tfx.tb_angles(0,0,180)
    rarm.set_pose(curr_pose)
    
    target_pose = rarm.get_pose() + [0.5, 0.4, -0.3]
    target_pose.orientation *= tfx.tb_angles(25,-20,10)
    sim.plot_transform(target_pose.matrix)

    start_joints = rarm.get_joints()
    n_steps = 40
    joint_traj = p.get_return_from_grasp_joint_trajectory(start_joints, target_pose, n_steps=n_steps)
    
    poses = [tfx.pose(rarm.fk(joints)) for joints in joint_traj]
    for pose in poses:
        sim.plot_transform(pose.matrix)
        
    print('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    #test_grasp_planner()
    test_return_from_grasp_planner()