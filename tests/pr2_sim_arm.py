import numpy as np
import random

from pr2_sim import arm, simulator
from utils import utils

import IPython

def test_ik():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')

    pose = rarm.get_pose()
    position = pose.position
    
    while True:     
        point = pose.position + np.array([np.random.uniform(0,1), np.random.uniform(-.3,.3), np.random.uniform(-.5,-.2)])
        sim.plot_point(sim.transform_from_to(point.array, 'base_link', 'world'), color=(0,1,0), size=.1)
     
        joints = rarm.ik_lookat(position, point)
        if joints is not None:
            rarm.set_joints(joints)
        else:
            print('Joints is None')
 
        sim.plot_transform(sim.transform_from_to(np.array(rarm.get_pose().matrix), 'base_link', 'world'))
        raw_input()
        sim.clear_plots()
        
def test_gripper_center_frame():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    print('Showing r_gripper_tool_frame')
    sim.plot_transform(sim.robot.GetLink('r_gripper_tool_frame').GetTransform())
    raw_input()
    
    print('Showing r_gripper_center_frame')
    sim.plot_transform(sim.robot.GetLink('r_gripper_center_frame').GetTransform())
    raw_input()
    
def test_kinect():
    sim = simulator.Simulator(view=True)
    
    larm = arm.Arm('left',sim=sim)
    rarm = arm.Arm('right',sim=sim)
    sim.add_box(rarm.get_pose(), [.5,.5,.5])
    
    larm.set_posture('mantis')
    rarm.set_posture('mantis')
    
    
    manip_links = [l for l in sim.robot.GetLinks() if l not in rarm.manip.GetIndependentLinks()]
    
    while True:
        print('Press "q" to exit')
        if (utils.Getch.getch() == 'q'):
            break
        
        rarm.teleop()
        is_coll = max([sim.env.CheckCollision(l) for l in manip_links])
        print('Is in collision: {0}\n'.format(is_coll))
        
            

if __name__ == '__main__':
    #test_ik()
    #test_gripper_center_frame()
    test_kinect()