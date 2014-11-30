import numpy as np
import random

import roslib
roslib.load_manifest('tfx')
import tfx

from pr2_sim import simulator

import IPython

def test_box():
    sim = simulator.Simulator(view=True)
    
    pose = tfx.pose([1,0,0], tfx.tb_angles(0,45,0))
    extents = [0.1,0.1,0.1]
    sim.add_box(pose.matrix, extents)
    
    print('Press enter to exit')
    raw_input()

if __name__ == '__main__':
    test_box()