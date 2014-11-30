import rospy, roslib
roslib.load_manifest('tfx')
import sensor_msgs.msg as sm
import geometry_msgs.msg as gm
import tfx

import numpy as np

from pr2 import convexify_pointcloud, planner
from pr2_sim import simulator, arm, camera

import IPython

class GraspPlan:
    def __init__(self):
        self.pc = None
        self.pc_sub = rospy.Subscriber('/cloud_pcd', sm.PointCloud2, self._pc_callback)
        
        self.handle_pose = None
        self.handle_pose_sub = rospy.Subscriber('/handle_detector/avg_handle_poses',
                                                gm.PoseArray, self._avg_handle_poses_callback)
        
        self.sim = simulator.Simulator(view=True)
        self.arm = arm.Arm('right', sim=self.sim)
        self.cam = camera.Camera(self.arm, self.sim)
        
        self.planner = planner.Planner('right', sim=self.sim)
        
    def _pc_callback(self, msg):
        self.pc = msg
        
    def _avg_handle_poses_callback(self, msg):
        min_dist, handle_pose = np.inf, None
        for pose in msg.poses:
            dist = tfx.pose(pose).position.norm
            if dist < min_dist:
                min_dist = dist
                handle_pose = tfx.pose(pose, header=msg.header)
        self.handle_pose = handle_pose
        
    def run(self):
        self.sim.update()
        
        while self.handle_pose is None and not rospy.is_shutdown():
            print('Waiting for average handle pose')
            rospy.sleep(1)
        handle_pose = self.handle_pose
        while True:
            try:
                handle_pose.stamp = rospy.Time.now()
                tmp_handle_pose = tfx.convertToFrame(handle_pose, '/base_link')
                handle_pose = tmp_handle_pose
                break
            except:
                pass
        
        self.sim.plot_transform(self.sim.transform_from_to(handle_pose, handle_pose.frame, 'world'))
        print('Received handle pose')
        
        while self.pc is None and not rospy.is_shutdown():
            print('Waiting for pointcloud')
            rospy.sleep(1)
            
        convexify_pointcloud.add_convexified_pointcloud_to_env(self.sim, self.pc, self.cam.get_pose())
        
        print('Convexified pointcloud, press enter')
        raw_input()
            
        joints = self.planner.get_joint_trajectory(self.arm.get_joints(), handle_pose, ignore_orientation=True)
        poses = [tfx.pose(self.arm.fk(joint), 'base_link') for joint in joints]
        for pose in poses:
            self.sim.plot_transform(self.sim.transform_from_to(pose, pose.frame, 'world'))
            
        IPython.embed()
        

if __name__ == '__main__':
    rospy.init_node('test_convexify_pointcloud', anonymous=True)
    
    grasp_plan = GraspPlan()
    rospy.sleep(1)
    grasp_plan.run()
    
#     IPython.embed()
    
    print('Press enter to exit')
    raw_input()