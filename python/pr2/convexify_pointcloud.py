import roslib, roslib.message
import sensor_msgs.msg as sm

import numpy as np
import struct

import openravepy as rave
import cloudprocpy

from pr2_sim import simulator

import IPython

_DATATYPES = {}
_DATATYPES[sm.PointField.INT8] = ('b', 1)
_DATATYPES[sm.PointField.UINT8] = ('B', 1)
_DATATYPES[sm.PointField.INT16] = ('h', 2)
_DATATYPES[sm.PointField.UINT16] = ('H', 2)
_DATATYPES[sm.PointField.INT32] = ('i', 4)
_DATATYPES[sm.PointField.UINT32] = ('I', 4)
_DATATYPES[sm.PointField.FLOAT32] = ('f', 4)
_DATATYPES[sm.PointField.FLOAT64] = ('d', 8)

def add_convexified_pointcloud_to_env(sim, pc2, transform=np.eye(4), num_cd_components=20, point_cloud_filter=lambda point: True):
    """
    Convexifies point cloud and adds to openrave environment
    
    :param sim: pr2_sim.simulator.Simulator
    :param pc2: the point cloud to read from
    :type  pc2: sensor_msgs.PointCloud2
    :param transform: 4x4 np.ndarray of transform for cloud in frame base_link
    :param point_cloud_filter: True if keep point
    """
    transform_world = sim.transform_from_to(transform, 'base_link', 'world')
    
    full_cloud = pc2_to_cloudprocpy(pc2, transform_world, point_cloud_filter)
    cloud = cloudprocpy.downsampleCloud(full_cloud, .005) # .005

    big_mesh = generate_mesh(cloud)
    convex_meshes = cloudprocpy.convexDecompHACD(big_mesh, num_cd_components)
    for i, mesh in enumerate(convex_meshes):
        sim.add_kinbody(mesh.getVertices(), mesh.getTriangles(), name='mesh_{0}'.format(i), check_collision=True)

def pc2_to_cloudprocpy(pc2, transform_world, point_cloud_filter):
    """
    :param pc2: the point cloud to read from
    :type  pc2: sensor_msgs.PointCloud2
    :param cam_pose_world: 4x4 np.ndarray of transform for cloud in frame world
    """
    points_gen = read_points(pc2, skip_nans=False)
    
    rot = transform_world[:3,:3]
    trans = transform_world[:3,3]
    points = list()
    for pt in points_gen:
        if point_cloud_filter(pt):
            pt = list(np.dot(rot, pt) + trans)
            pt.append(1)
            points.append(pt)
    
    
    # reorganize cloud data to construct a cloud object
    n_dim = 4 # 3 spatial dimension + 1 for homogenity
    cloud = cloudprocpy.CloudXYZ()
    xyz1 = np.array(points)
    cloud.from2dArray(xyz1)
    
    return cloud

def read_points(cloud, field_names=None, skip_nans=False, uvs=[]):
    """
    Read points from a L{sensor_msgs.PointCloud2} message.

    :param cloud: the point cloud to read from.
    :type  cloud: sensor_msgs.PointCloud2
    :param field_names: the names of fields to read. If None, read all fields. [default: None]
    :type  field_names: iterable
    :param skip_nans: if True, then don't return any point with a NaN value.
    :type  skip_nans: bool [default: False]
    :param uvs: if specified, then only return the points at the given coordinates. [default: empty list]
    :type  uvs: iterable
    :return: generator which yields a list of values for each point.
    :rtype:  generator
    """
    assert isinstance(cloud, roslib.message.Message) and cloud._type == 'sensor_msgs/PointCloud2', 'cloud is not a sensor_msgs.msg.PointCloud2'
    fmt = _get_struct_format(cloud.is_bigendian, cloud.fields, field_names)
    width, height, point_step, row_step, data, isnan = cloud.width, cloud.height, cloud.point_step, cloud.row_step, cloud.data, np.isnan
    unpack_from = struct.Struct(fmt).unpack_from

    if skip_nans:
        if uvs:
            for u, v in uvs:
                p = unpack_from(data, (row_step * v) + (point_step * u))
                has_nan = False
                for pv in p:
                    if isnan(pv):
                        has_nan = True
                        break
                if not has_nan:
                    yield p
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    p = unpack_from(data, offset)
                    has_nan = False
                    for pv in p:
                        if isnan(pv):
                            has_nan = True
                            break
                    if not has_nan:
                        yield p
                    offset += point_step
    else:
        if uvs:
            for u, v in uvs:
                yield unpack_from(data, (row_step * v) + (point_step * u))
        else:
            for v in xrange(height):
                offset = row_step * v
                for u in xrange(width):
                    yield unpack_from(data, offset)
                    offset += point_step

def _get_struct_format(is_bigendian, fields, field_names=None):
    fmt = '>' if is_bigendian else '<'

    offset = 0
    for field in (f for f in sorted(fields, key=lambda f: f.offset) if field_names is None or f.name in field_names):
        if offset < field.offset:
            fmt += 'x' * (field.offset - offset)
            offset = field.offset
        if field.datatype not in _DATATYPES:
            print >> sys.stderr, 'Skipping unknown PointField datatype [%d]' % field.datatype
        else:
            datatype_fmt, datatype_length = _DATATYPES[field.datatype]
            fmt    += field.count * datatype_fmt
            offset += field.count * datatype_length

    return fmt

def generate_mesh(cloud, decimation_rate=0.5):
    """
    For unordered point cloud, calculates normals and then creates mesh
    
    :param cloud: cloudprocpy cloud
    :return pcl polygon mesh
    """
#     import warnings
#     warnings.filterwarnings('error')
    
    cloud_with_normals = cloudprocpy.mlsAddNormals(cloud, .02)
    filtered_cloud = list()
    for pt_and_normal in cloud_with_normals.to2dArray():
        normal = pt_and_normal[3:]
#         try:
        if np.linalg.norm(normal) < 1e4:
            filtered_cloud.append(pt_and_normal)
#         except Warning as e:
#             import IPython
#             IPython.embed()
    cloud_with_normals.from2dArray(np.array(filtered_cloud))

    big_mesh = cloudprocpy.meshGP3(cloud_with_normals, search_radius=0.02) # 0.04
#     simple_mesh = cloudprocpy.quadricSimplifyVTK(big_mesh, decimation_rate) # decimate mesh with VTK function
#     return simple_mesh
    return big_mesh

