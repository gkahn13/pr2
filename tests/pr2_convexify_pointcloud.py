import rospy
import sensor_msgs.msg as sm

from pr2 import convexify_pointcloud
from pr2_sim import simulator, arm, camera

import IPython

class TestConvexifyPointcloud:
    def __init__(self):
        self.pc = None
        self.pc_sub = rospy.Subscriber('/cloud_pcd', sm.PointCloud2, self._pc_callback)
        
        self.sim = simulator.Simulator(view=True)
        self.rarm = arm.Arm('right', sim=self.sim)
        self.larm = arm.Arm('left', sim=self.sim)
        self.cam = camera.Camera(self.arm, self.sim)
        
        self.rarm.set_posture('mantis')
        self.larm.set_posture('mantis')

    def _pc_callback(self, msg):
        self.pc = msg
        
    def run(self):
        while self.pc is None and not rospy.is_shutdown():
            print('Waiting for pointcloud')
            rospy.sleep(1)
            
        convexify_pointcloud.add_convexified_pointcloud_to_env(self.sim, self.pc, self.cam.get_pose())
        

if __name__ == '__main__':
    rospy.init_node('test_convexify_pointcloud', anonymous=True)
    
    test = TestConvexifyPointcloud()
    test.run()
    
    print('Press enter to exit')
    raw_input()