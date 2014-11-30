import rospy

from pr2 import arm
from pr2_sim import simulator

def test_mantis():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    larm = arm.Arm('left', sim=sim)
    
    rarm.go_to_posture('mantis', block=False)
    larm.go_to_posture('mantis', block=False)
    
    rarm.open_gripper()
    larm.open_gripper()
    
def test_gripper():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    larm = arm.Arm('left', sim=sim)
    
#     rarm.close_gripper()
#     larm.close_gripper()
    
    rarm.open_gripper()
    larm.open_gripper()
    
def test_joints():
    sim = simulator.Simulator(view=True)
    rarm = arm.Arm('right', sim=sim)
    larm = arm.Arm('left', sim=sim)
    
    print('rarm joints: {0}'.format(rarm.get_joints()))

if __name__ == '__main__':
    rospy.init_node('pr2_arm', anonymous=True)
#     test_mantis()
    test_gripper()
#     test_joints()