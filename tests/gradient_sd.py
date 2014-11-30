"""
Test walking along the gradient of the signed-distance
of a point to the view frustum
with respect to the arm joints
"""

import time

import numpy as np

from pr2_sim import arm, camera, simulator
from geometry import geometry3d
from utils import utils

import IPython

def signed_distance(j, point, triangles3d, rarm, cam, plot=False):
    rarm.set_joints(j)
    truncated_frustum = cam.truncated_view_frustum(triangles3d, plot=plot)
    return cam.signed_distance(point, truncated_frustum)

def signed_distance_grad(j, point, triangles3d, rarm, cam, step=1e-5):
    grad = np.zeros(j.shape)
    
    j_p, j_m = np.array(j), np.array(j)
    for i in xrange(len(j)):
        j_p[i] = j[i] + step
        j_m[i] = j[i] - step
        
        sd_p = signed_distance(j_p, point, triangles3d, rarm, cam)
        sd_m = signed_distance(j_m, point, triangles3d, rarm, cam)
        
        grad[i] = (sd_p - sd_m) / (2*step)
        
        j_p[i] = j[i]
        j_m[i] = j[i]
        
    return grad

def fast_signed_distance_grad(j, point, triangles3d, rarm, cam, step=1e-5):
    grad = np.zeros(j.shape)
    
    rarm.set_joints(j)
    cam_pose = np.array(cam.get_pose())
    cam_pose_inv = np.linalg.inv(cam_pose)
    
    truncated_frustum = cam.truncated_view_frustum(triangles3d, plot=False)
    
    sd_pyramid3d, sd = None, np.inf
    for pyramid3d in truncated_frustum:
        sd_tmp = pyramid3d.signed_distance(point)
        if sd_tmp < sd:
            sd = sd_tmp
            sd_pyramid3d = pyramid3d
            
    rel_points, rel_point_frames = list(), list()
    for sd_line3d in [geometry3d.Line(s.p0, s.p1) for s in sd_pyramid3d.side_segments]:
#         print('\nsd_line3d')
#         sd_line3d.plot(cam.sim, frame='base_link', color=(0,1,0))
        found_clipping_tri3d = False
        for tri3d in triangles3d:
            for tri_line3d in [geometry3d.Line(s.p0, s.p1) for s in tri3d.segments]:
#                 print('tri_line3d')
#                 tri_line3d.plot(cam.sim, frame='base_link', color=(0,0,1))
                dist = tri_line3d.distance_to(sd_line3d)
#                 print('dist: {0}'.format(dist))
                if dist < 1e-3:
#                     print('lines touch!')
                    closest_point = tri_line3d.closest_points(sd_line3d)[0]
                    found_clipping_tri3d = True
#                 raw_input()
#                 cam.sim.clear_plots(1)
                if found_clipping_tri3d:
                    break
            if found_clipping_tri3d:
                break
        
#         raw_input()
#         cam.sim.clear_plots(1)
        
        if found_clipping_tri3d:
            rel_points.append(closest_point)
            rel_point_frames.append('base_link')
        else:
            rel_points.append(cam_pose_inv[:3,:3].dot(sd_line3d.p1)+cam_pose_inv[:3,3])
            rel_point_frames.append('camera_rgb_optical_frame')
            
#     print('rel_points: {0}'.format(rel_points))
#     print('rel_point_frames: {0}'.format(rel_point_frames))
            
    relative_pyramid = camera.RelativePyramid(rel_points, rel_point_frames)
#     abs_pyramid = relative_pyramid.construct_pyramid(cam)
#     abs_pyramid.plot(cam.sim, frame='base_link', fill=False, with_sides=True, color=(1,0,0), alpha=0.25)
    
#     IPython.embed()
#     import sys
#     sys.exit()
    
    j_p, j_m = np.array(j), np.array(j)
    for i in xrange(len(j)):
        j_p[i] = j[i] + step
        j_m[i] = j[i] - step
        
        rarm.set_joints(j_p)
        sd_p = relative_pyramid.signed_distance(cam, point)
        rarm.set_joints(j_m)
        sd_m = relative_pyramid.signed_distance(cam, point)
        
        grad[i] = (sd_p - sd_m) / (2*step)
        
        j_p[i] = j[i]
        j_m[i] = j[i]
        
    return grad

def test_gradient_sd():
    sim = simulator.Simulator(view=True)
    larm = arm.Arm('left',sim=sim)
    larm.set_posture('mantis')
    rarm = arm.Arm('right',sim=sim)
    rarm.set_posture('mantis')
    
    cam = camera.Camera(rarm, sim)
#     triangles3d = []
#     triangles3d = [geometry3d.Triangle([.7,0,.8],[.7,0,1.1],[.7,-.3,.7])]
#     triangles3d = [geometry3d.Triangle([.5,0,.5],[.8,0,.6],[.5,-.3,.9])]
#     triangles3d = [geometry3d.Triangle([np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)],
#                                        [np.random.uniform(.2,.5), np.random.uniform(-.5,0), np.random.uniform(.25,.75)]) for _ in xrange(3)]
    table_center = np.array([.2,.7,.5])
    triangles3d = [#geometry3d.Triangle(table_center, table_center+np.array([.5,-1.4,0]), table_center+np.array([.5,0,0])),
                   #geometry3d.Triangle(table_center, table_center+np.array([0,-1.4,0]), table_center+np.array([.5,-1.4,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-.7,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,0])),
                   geometry3d.Triangle(table_center+np.array([.25,-1.2,0]), table_center+np.array([.25,-.7,.2]), table_center+np.array([.25,-1.2,.2]))]
    
#     point = np.array([0.5, 0.7, 0.66]) # sd: 0.115138180967
#     point = np.array([0.5, -2, 0.66])
    point = np.array([0.640, -0.170,  0.560]) # sd: 0.0189611173538
    j = rarm.get_joints()
    
    while True:
        sim.clear_plots()
        sim.plot_point(sim.transform_from_to(point, 'base_link', 'world'), size=.02, color=(0,0,1))
        cam.plot(frame='base_link')
        for tri3d in triangles3d:
            tri3d.plot(sim, frame='base_link', fill=True)
        
        sd = 0 #signed_distance(j, point, triangles3d, rarm, cam, plot=False)
        start = time.time()
        sd_grad = signed_distance_grad(j, point, triangles3d, rarm, cam)
        elapsed = time.time() - start
        
        start = time.time()
        sd_grad_fast = fast_signed_distance_grad(j, point, triangles3d, rarm, cam)
        fast_elapsed = time.time() - start 
        
        print('sd: {0}'.format(sd))
        print('sd_grad: {0}'.format(sd_grad))
        print('sd_grad_fast: {0}'.format(sd_grad_fast))
        print('sd_grad diff: {0}'.format(np.linalg.norm(sd_grad - sd_grad_fast)))
        print('sd_grad time: {0}'.format(elapsed))
        print('sd_grad_fast time: {0}'.format(fast_elapsed))
        
        print('Press enter to continue (or "q" to exit)')
        char = utils.Getch.getch()
        if char == 'q':
            break
        
        j -= (np.pi/64.)*(sd_grad/np.linalg.norm(sd_grad))
        rarm.set_joints(j)
    

if __name__ == '__main__':
    test_gradient_sd()