import roslib
roslib.load_manifest('tfx')
import tfx

import numpy as np
from matplotlib import pyplot as plt
from matplotlib import delaunay

import time
import random

from geometry import geometry2d, geometry3d
from pr2_sim import simulator, arm
from utils import utils

wrist_to_hand = tfx.transform((-0.111015481946, -0.0160376173344, -0.0735991062825),
                              (0.531327739419, 0.463547417, 0.524778479206, 0.476888009158))

class RelativePyramid:
    def __init__(self, points, point_frames):
        assert(len(points) == 3 and len(point_frames) == 3)
        self.points = points
        self.point_frames = point_frames
        
    def construct_pyramid(self, cam):
        cam_pose = np.array(cam.get_pose().matrix)
        cam_rot, cam_trans = cam_pose[:3,:3], cam_pose[:3,3]
        
        abs_points3d = list()
        for pt, frame in zip(self.points, self.point_frames):
            assert(frame.count('base_link') > 0 or frame.count('camera') > 0)
            
            if frame.count('camera') > 0:
                pt = cam_rot.dot(pt.T) + cam_trans
                
            abs_seg3d = cam.segment_through_pixel(cam.pixel_from_point(tfx.point(pt,frame='base_link')))
            abs_points3d += [abs_seg3d.p0, abs_seg3d.p1]

        pyramid3d = geometry3d.TruncatedPyramid(*abs_points3d)
        return pyramid3d
    
    def signed_distance(self, cam, point):
        return self.construct_pyramid(cam).signed_distance(point)

class Camera:
    def __init__(self, arm, sim, tool_to_camera=None,
                 height=480., width=640., focal_length=.01,
                 fx=525., fy=525., cx=319.5, cy=239.5, min_range=.3, max_range=1.5):
                 #fx=640.*2., fy=480.*2., cx=640./2.+0.5, cy=480./2.+0.5, max_range=1.5):
        self.arm = arm
        self.sim = sim
        self.tool_to_camera = tool_to_camera if tool_to_camera is not None else tfx.transform(wrist_to_hand)
        
        self.height = height
        self.width = width
        
        self.fx, self.fy, self.cx, self.cy = fx, fy, cx, cy
        
        self.P = np.array([[fx,  0, cx],
                           [ 0, fy, cy],
                           [ 0,  0,  1]])
        
        self.focal_length = focal_length # TODO: not sure if needed
        self.min_range = min_range
        self.max_range = max_range
        self.height_m = focal_length*(height/fy)
        self.width_m = focal_length*(width/fx)
        
    ##############
    # state info #
    ##############
                
    def get_pose(self):
        """
        :return tfx.pose from frame 'base_link'
        """
        return tfx.pose(self.sim.transform_from_to(self.tool_to_camera.matrix, self.arm.tool_frame, 'base_link'), frame='base_link')
        
    #########################
    # camera matrix methods #
    #########################
        
    def pixel_from_point(self, point):
        """
        Projects point onto image plane
        
        :param point: tfx.pose/tfx.point
        :return 2d np.array (even if outside of image plane)
        """
        assert point.frame == 'base_link'
        
        cam_pose_mat = np.array(self.get_pose().as_tf().matrix)
        point_pose_mat = np.array(point.as_pose().matrix)
        point_transform_cam = np.linalg.solve(cam_pose_mat, point_pose_mat)
        y = np.dot(self.P, point_transform_cam[:3,3])
        
        pixel = np.array([y[1]/float(y[2]), y[0]/float(y[2])])
        return pixel
        
    def segment_through_pixel(self, pixel, start_dist=None):
        """
        Returns segment from camera origin through pixel
        
        :param pixel: 2d list/np.array
        :return geometry3d.Segment (with endpoints in frame 'base_link')
        """
        # assert (0 <= pixel[0] <= self.height) and (0 <= pixel[1] <= self.width)
        start_dist = start_dist if start_dist is not None else self.min_range
        
        pixel = np.array(pixel)
        pixel_centered = pixel - np.array([self.height/2., self.width/2.])
        
        pixel3d_centered_m_min = start_dist*np.array([pixel_centered[1]/self.fx,
                                                      pixel_centered[0]/self.fy,
                                                      1])
        pixel3d_centered_m_max = self.max_range*np.array([pixel_centered[1]/self.fx,
                                                         pixel_centered[0]/self.fy,
                                                         1])
        
        transform = self.get_pose().as_tf()
        p0 = (transform*tfx.pose(pixel3d_centered_m_min)).position.array 
        p1 = (transform*tfx.pose(pixel3d_centered_m_max)).position.array        
        
        return geometry3d.Segment(p0, p1)
    
    #####################
    # calculate frustum #
    #####################
    
    def truncated_view_frustum(self, triangles3d, plot=False):
        """
        Truncates the view frustum against environment triangles
        
        :param triangles3d: list of geometry3d.Triangle with points in frame 'base_link'
        :param plot: if True, plots in 3d (OpenRAVE) and 2d (matplotlib)
        :return list of geometry3d.Pyramid
        """
        start_time = time.time()
        
        seg3d_a = self.segment_through_pixel([0,self.width])
        seg3d_b = self.segment_through_pixel([0,0])
        seg3d_c = self.segment_through_pixel([self.height,0])
        seg3d_d = self.segment_through_pixel([self.height,self.width])
        
        frustum = geometry3d.ViewFrustum(seg3d_a.p0, seg3d_a.p1,
                                         seg3d_b.p0, seg3d_b.p1,
                                         seg3d_c.p0, seg3d_c.p1,
                                         seg3d_d.p0, seg3d_d.p1)
        
        clipped_triangles3d = list()
        for tri3d in triangles3d:
            clipped_triangles3d += frustum.clip_triangle(tri3d)
            
        triangles2d = self.project_triangles(clipped_triangles3d)
        
        segments2d = set()
        for tri2d in triangles2d:
            segments2d.update(tri2d.segments)
        
        # avoid redundant points
        points2d = {geometry2d.Point([0,0]),
                    geometry2d.Point([self.height,0]),
                    geometry2d.Point([0,self.width]),
                    geometry2d.Point([self.height,self.width])}
        # add vertices
        for seg2d in segments2d:
            points2d.add(geometry2d.Point(seg2d.p0))
            points2d.add(geometry2d.Point(seg2d.p1))
        # add intersections
        segments2d_list = list(segments2d)
        for i in xrange(len(segments2d_list)-1):
            seg2d = segments2d_list[i]
            for j in xrange(i+1, len(segments2d_list)):
                other_seg2d = segments2d_list[j]
                intersection = seg2d.intersection(other_seg2d)
                if intersection is not None:
                    points2d.add(geometry2d.Point(intersection))
                    
        partition_triangles2d = set()
        for pt2d in points2d:
            # find other points that don't cross anything in segments2d
            p = pt2d.p
            points_in_los = set()
            for other_pt2d in points2d:
                other_p = other_pt2d.p
                if pt2d != other_pt2d:
                    seg2d = geometry2d.Segment(p, other_p)
                    for check_seg2d in segments2d:
                        intersection = seg2d.intersection(check_seg2d)
                        if intersection is not None and not seg2d.is_endpoint(intersection):
                            break
                    else:
                        points_in_los.add(geometry2d.Point(other_p)) 
                                
            # sort segments by angle
            seg2d_compare = geometry2d.Segment([0,0], [-1,0])
            segments2d_in_los_sorted = sorted([geometry2d.Segment(p, other.p) for other in points_in_los], key=lambda seg: seg.angle(seg2d_compare))
                
            new_partition_triangles2d = set()
            for i in xrange(len(segments2d_in_los_sorted)-1):
                tri_seg2d = geometry2d.Segment(segments2d_in_los_sorted[i].p1, segments2d_in_los_sorted[i+1].p1)
                # check top segment of triangle against segments2d
                for check_seg2d in segments2d:
                    intersection = tri_seg2d.intersection(check_seg2d)
                    if intersection is not None and not tri_seg2d.is_endpoint(intersection):
                        break
                else:        
                    tri2d = geometry2d.Triangle(p, tri_seg2d.p0, tri_seg2d.p1)
                    if tri2d.area > geometry2d.epsilon:
                        new_partition_triangles2d.add(tri2d)
            
            # update partition and new segments
            partition_triangles2d.update(new_partition_triangles2d)
            for tri2d in new_partition_triangles2d:
                segments2d.update(tri2d.segments)
                
            if sum([tri2d.area for tri2d in partition_triangles2d]) >= self.width*self.height:
                break
            
        start_time_3d = time.time()
        # now that we have tesselated the projection into triangles
        # find out which 3d triangle each projection belongs to
        # by shooting out rays through the 2d triangles
        camera_position = self.get_pose().position.array
        pyramids3d = list()
        for tri2d in partition_triangles2d:
            tri2d_vertices = tri2d.vertices
            center2d = sum(tri2d_vertices)/3.
            
            center_seg3d = self.segment_through_pixel(center2d, 0.0) # start at origin
            vertices_seg3d = [self.segment_through_pixel(vertex) for vertex in tri2d_vertices]
            
            min_dist, min_tri3d = np.inf, None
            for tri3d in triangles3d:
                intersection = tri3d.intersection(center_seg3d)
                if intersection is not None:
                    dist = np.linalg.norm(intersection - camera_position)
                    if dist < min_dist:
                        min_dist = dist
                        min_tri3d = tri3d
                    
            if min_tri3d is None:
                pyramids3d.append(geometry3d.TruncatedPyramid(vertices_seg3d[0].p0, vertices_seg3d[0].p1,
                                                              vertices_seg3d[1].p0, vertices_seg3d[1].p1,
                                                              vertices_seg3d[2].p0, vertices_seg3d[2].p1))
            elif min_dist >= self.min_range:
                tri3d_intersections = [min_tri3d.closest_point_on_segment(vertex_seg3d) for vertex_seg3d in vertices_seg3d]
                assert len(filter(lambda x: x is None, tri3d_intersections)) == 0
                pyramids3d.append(geometry3d.TruncatedPyramid(vertices_seg3d[0].p0, tri3d_intersections[0],
                                                              vertices_seg3d[1].p0, tri3d_intersections[1], 
                                                              vertices_seg3d[2].p0, tri3d_intersections[2]))
                
        if plot:
            end_time = time.time()
            total_time = time.time() - start_time
            time_3d = end_time - start_time_3d
            time_2d = start_time_3d - start_time
            print('Total time: {0}'.format(total_time))
            print('2d time: {0}'.format(time_2d))
            print('3d time: {0}'.format(time_3d))
            total_area = sum([tri2d.area for tri2d in partition_triangles2d])
            print('Total area (should be {0}): {1}'.format(self.width*self.height, total_area))
            print('Number of points: {0}'.format(len(points2d)))
            print('Number of triangles: {0}'.format(len(partition_triangles2d)))
                
            self.plot(frame='base_link')
            for tri3d in triangles3d:
                tri3d.plot(self.sim, frame='base_link', fill=True, color=(0,0,1))
                
            for pyramid in pyramids3d:
                pyramid.plot(self.sim, frame='base_link', fill=True, with_sides=True, color=(0,1,0), alpha=1)
            
            plt.close(1)
            fig = plt.figure(1)
            axes = fig.add_subplot(111)
            
            for tri2d in triangles2d:
                for segment in tri2d.segments:
                    p0, p1 = segment.p0, segment.p1
                    p0_flip = [p0[1], self.height - p0[0]]
                    p1_flip = [p1[1], self.height - p1[0]]
                    axes.plot([p0_flip[0], p1_flip[0]], [p0_flip[1], p1_flip[1]], 'b--o', linewidth=2.0)
                 
            axes.plot([0, 0, self.width, self.width, 0], [0, self.height, self.height, 0, 0], 'b--o', linewidth=2.0)
            
            for pt2d in points2d:
                axes.plot(pt2d.p[1], self.height - pt2d.p[0], 'rx', markersize=10.0)
                
            plt.xlim((-10, self.width+10))
            plt.ylim((-10, self.height+10))
            
            colors = plt.cm.jet(np.linspace(0, 1, len(partition_triangles2d)))
            for i, tri2d in enumerate(partition_triangles2d):
                x = [p[1] for p in tri2d.vertices]
                y = [self.height - p[0] for p in tri2d.vertices]
                axes.fill(x, y, color=colors[i], edgecolor=(1,1,1))
            
            plt.show(block=False)
            
        return pyramids3d
            
        
    def truncated_view_frustum_regions(self, triangles3d, plot=False):
        start_time = time.time()
        
        seg3d_a = self.segment_through_pixel([0,self.width])
        seg3d_b = self.segment_through_pixel([0,0])
        seg3d_c = self.segment_through_pixel([self.height,0])
        seg3d_d = self.segment_through_pixel([self.height,self.width])
        
        frustum = geometry3d.ViewFrustum(seg3d_a.p0, seg3d_a.p1,
                                         seg3d_b.p0, seg3d_b.p1,
                                         seg3d_c.p0, seg3d_c.p1,
                                         seg3d_d.p0, seg3d_d.p1)
        
        clipped_triangles3d = list()
        for tri3d in triangles3d:
            clipped_triangles3d += frustum.clip_triangle(tri3d)
            
        triangles2d = self.project_triangles(clipped_triangles3d)
        
        segments2d = {geometry2d.Segment([0,0],[self.height,0]),
                      geometry2d.Segment([self.height,0],[self.height,self.width]),
                      geometry2d.Segment([self.height,self.width],[0,self.width]),
                      geometry2d.Segment([0,self.width],[0,0])}
        for tri2d in triangles2d:
            segments2d.update(tri2d.segments)
            
        # form vertical lines from vertices    
        vert_lines2d = {0, self.width} # store width values
        for seg2d in segments2d:
            for other_seg2d in segments2d:
                if seg2d != other_seg2d:
                    intersection = seg2d.intersection(other_seg2d)
                    if intersection is not None:
                        vert_lines2d.add(intersection[1])
                    
        vert_lines2d_sorted = sorted(vert_lines2d, key=lambda l: l)
        
        regions2d = [[] for width in vert_lines2d_sorted[:-1]]
        for seg2d in segments2d:
            p0, p1 = seg2d.p0, seg2d.p1
            if p1[1] < p0[1]:
                p0, p1 = p1, p0
                
            start, end = vert_lines2d_sorted.index(p0[1]), vert_lines2d_sorted.index(p1[1])
            
            start_width = vert_lines2d_sorted[start]
            end_width = vert_lines2d_sorted[end]
            total_width = end_width - start_width
            curr_point = p0
            for i in xrange(start, end):
                next_width = vert_lines2d_sorted[i+1]
                t = (next_width - start_width) / total_width
                next_point = t*(p1-p0) + p0
                regions2d[i].append(geometry2d.Segment(curr_point, next_point))
                curr_point = next_point

        for i, region2d in enumerate(regions2d):
            regions2d[i] = sorted(region2d, key=lambda seg2d: (seg2d.p0[0] + seg2d.p1[0])/2.0)
            
#         triangle_regions2d = [list() for region2d in regions2d]
#         for region2d, triangle_region2d in zip(regions2d, triangle_regions2d):
#             for i in xrange(len(region2d)-1):
#                 curr_seg2d = region2d[i]
#                 next_seg2d = region2d[i+1]
#                 # know curr_seg2d and next_seg2d go from left to right
#                 tri2d_0 = geometry2d.Triangle(curr_seg2d.p0, curr_seg2d.p1, next_seg2d.p0)
#                 tri2d_1 = geometry2d.Triangle(next_seg2d.p0, next_seg2d.p1, curr_seg2d.p1)
#                 if tri2d_0.area > geometry2d.epsilon:
#                     triangle_region2d.append(tri2d_0)
#                 if tri2d_1.area > geometry2d.epsilon:
#                     triangle_region2d.append(tri2d_1)
            
        partition_triangles2d = list()
        for region2d in regions2d:
            for i in xrange(len(region2d)-1):
                curr_seg2d = region2d[i]
                next_seg2d = region2d[i+1]
                # know curr_seg2d and next_seg2d go from left to right
                tri2d_0 = geometry2d.Triangle(curr_seg2d.p0, curr_seg2d.p1, next_seg2d.p0)
                tri2d_1 = geometry2d.Triangle(next_seg2d.p0, next_seg2d.p1, curr_seg2d.p1)
                if tri2d_0.area > geometry2d.epsilon:
                    partition_triangles2d.append(tri2d_0)
                if tri2d_1.area > geometry2d.epsilon:
                    partition_triangles2d.append(tri2d_1)
                    
#         camera_position = self.get_pose().position.array
#         pyramids3d = list()
#         for i, triangle_region2d in enumerate(triangle_regions2d):
#             lower, upper = vert_lines_2d_sorted[i], vert_lines2d_sorted[i+1]
#             
                    
        start_time_3d = time.time()
        # now that we have tesselated the projection into triangles
        # find out which 3d triangle each projection belongs to
        # by shooting out rays through the 2d triangles
        camera_position = self.get_pose().position.array
        pyramids3d = list()
        for tri2d in partition_triangles2d:
            tri2d_vertices = tri2d.vertices
            center2d = sum(tri2d_vertices)/3.
             
            center_seg3d = self.segment_through_pixel(center2d, 0.0) # start at origin
            vertices_seg3d = [self.segment_through_pixel(vertex) for vertex in tri2d_vertices]
             
            min_dist, min_tri3d = np.inf, None
            for tri3d in triangles3d:
                intersection = tri3d.intersection(center_seg3d)
                if intersection is not None:
                    dist = np.linalg.norm(intersection - camera_position)
                    if dist < min_dist:
                        min_dist = dist
                        min_tri3d = tri3d
                     
            if min_tri3d is None:
                pyramids3d.append(geometry3d.TruncatedPyramid(vertices_seg3d[0].p0, vertices_seg3d[0].p1,
                                                              vertices_seg3d[1].p0, vertices_seg3d[1].p1,
                                                              vertices_seg3d[2].p0, vertices_seg3d[2].p1))
            elif min_dist >= self.min_range:
                tri3d_intersections = [min_tri3d.closest_point_on_segment(vertex_seg3d) for vertex_seg3d in vertices_seg3d]
                assert len(filter(lambda x: x is None, tri3d_intersections)) == 0
                pyramids3d.append(geometry3d.TruncatedPyramid(vertices_seg3d[0].p0, tri3d_intersections[0],
                                                              vertices_seg3d[1].p0, tri3d_intersections[1], 
                                                              vertices_seg3d[2].p0, tri3d_intersections[2]))
                
        if plot:
            end_time = time.time()
            total_time = time.time() - start_time
            time_3d = end_time - start_time_3d
            time_2d = start_time_3d - start_time
            print('Total time: {0}'.format(total_time))
            print('2d time: {0}'.format(time_2d))
            print('3d time: {0}'.format(time_3d))
            
            print('Number of regions: {0}'.format(len(regions2d)))
#             for i, region2d in enumerate(regions2d):
#                 print('Number segments in region {0}: {1}'.format(i, len(region2d)))
                
            print('Number of triangles: {0}'.format(len(partition_triangles2d)))
            
#             print('vert_lines2d_sorted')
#             for vert_line2d in vert_lines2d_sorted:
#                 print(vert_line2d)
#                 
#             print('regions2d')
#             for i, region2d in enumerate(regions2d):
#                 print('Region {0} size: {1}'.format(i, len(region2d)))
            
            plt.close(1)
            fig = plt.figure(1)
            axes = fig.add_subplot(111)
            
            plt.xlim((-10, self.width+10))
            plt.ylim((-10, self.height+10))
            
            for tri2d in triangles2d:
                for segment in tri2d.segments:
                    p0, p1 = segment.p0, segment.p1
                    p0_flip = [p0[1], self.height - p0[0]]
                    p1_flip = [p1[1], self.height - p1[0]]
                    axes.plot([p0_flip[0], p1_flip[0]], [p0_flip[1], p1_flip[1]], 'y--o', linewidth=3.0)
                    
            for width in vert_lines2d_sorted:
                axes.plot([width, width], [0, self.height], 'r')
                
#             for region2d in regions2d:
#                 colors = plt.cm.hsv(np.linspace(0, 1, len(region2d)))
#                 random.shuffle(colors)
#                 for seg2d, color in zip(region2d, colors):
#                     p0, p1 = seg2d.p0, seg2d.p1
#                     p0_flip = [p0[1], self.height - p0[0]]
#                     p1_flip = [p1[1], self.height - p1[0]]
#                     axes.plot([p0_flip[0], p1_flip[0]], [p0_flip[1], p1_flip[1]], '-x', color=color, linewidth=3.0)
                    
            colors = plt.cm.jet(np.linspace(0, 1, 10))
            for i, tri2d in enumerate(partition_triangles2d):
                x = [p[1] for p in tri2d.vertices]
                y = [self.height - p[0] for p in tri2d.vertices]
                axes.fill(x, y, color=colors[i%len(colors)], edgecolor=(1,1,1), linewidth=3.0)
#                 print('partition tri2d {0} area: {1}'.format(i, tri2d.area))
                
                
            plt.show(block=False)
            
            self.plot(frame='base_link')
            for tri3d in triangles3d:
                tri3d.plot(self.sim, frame='base_link', fill=True, color=(0,0,1))
                
            for pyramid in pyramids3d:
                pyramid.plot(self.sim, frame='base_link', fill=True, with_sides=True, color=(0,1,0), alpha=1)
        
        
    def project_triangles(self, triangles3d):
        """
        Projects 3d triangles onto image plane and returns 2d triangles
        
        :param triangles3d: list of geometry3d.Triangle with points in frame 'base_link'
        :return list of geometry2d.Triangle with points in frame 'base_link'
        """
        triangles2d = list()
        for triangle3d in triangles3d:
            a_proj = self.pixel_from_point(tfx.point(triangle3d.a, frame='base_link'))
            b_proj = self.pixel_from_point(tfx.point(triangle3d.b, frame='base_link'))
            c_proj = self.pixel_from_point(tfx.point(triangle3d.c, frame='base_link'))
            
            triangles2d.append(geometry2d.Triangle(a_proj, b_proj, c_proj))
            
        return triangles2d
                         
    ###################
    # signed distance #
    ###################
    
    def signed_distance(self, point, truncated_frustum):
        """
        Calculates signed-distance of point to truncated view frustum
        
        :param point: tfx.point/np.array
        :param truncated_frustum: list of geometry3d.Pyramid
        :return float signed-distance
        """
        point = tfx.point(point).array
        return min([pyramid3d.signed_distance(point) for pyramid3d in truncated_frustum])
    
    ##################
    # visualizations #
    ##################
        
    def plot(self, frame='world', fill=False, with_sides=True, color=(1,0,0), alpha=0.25):
        """
        :param frame: frame in which points are defined in
        :param with_sides: if True, plots side edges too
        :param color: (r,g,b) [0,1]
        """
        seg3d_a = self.segment_through_pixel([0,self.width])
        seg3d_b = self.segment_through_pixel([0,0])
        seg3d_c = self.segment_through_pixel([self.height,0])
        seg3d_d = self.segment_through_pixel([self.height,self.width])
        
        frustum = geometry3d.ViewFrustum(seg3d_a.p0, seg3d_a.p1,
                                         seg3d_b.p0, seg3d_b.p1,
                                         seg3d_c.p0, seg3d_c.p1,
                                         seg3d_d.p0, seg3d_d.p1)
        
        frustum.plot(self.sim, frame=frame, fill=fill, with_sides=with_sides, color=color, alpha=alpha)
            
    