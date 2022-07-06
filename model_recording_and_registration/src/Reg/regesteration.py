#!/usr/bin/env python3.8
import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as r

target_point=np.array([[-0.0040],[ -0.022],[0.032],[1]])

trans_g2b=np.array([[0.825], [-0.063], [-0.166],[1]])
quat=[0.165, -0.826, -0.022, 0.538]

pre_rot=r.from_quat(quat)
rot_g2b=np.array(pre_rot.as_matrix())

Transform=np.vstack([R,row])

voxel_size=0.05

pcd=o3d.io.read_point_cloud("cropped.ply")
downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)


downpcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))
pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        downpcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))


stl=o3d.io.read_triangle_mesh("Skeleton_T.stl")
skeleton=stl.sample_points_uniformly(len(pcd.points))


R = skeleton.get_rotation_matrix_from_xyz((0, np.pi, -np.pi/3))
skeleton.rotate(R, center=(0, 0, 0))
downstl=skeleton.voxel_down_sample(voxel_size=voxel_size)


downstl.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))

stl_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        downstl,
        o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))


distance_threshold=voxel_size*1.5

result_global= o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        downstl,downpcd, stl_fpfh,pcd_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            maximum_correspondence_distance=distance_threshold))



result = o3d.pipelines.registration.registration_icp(
        downstl, downpcd, distance_threshold, result_global.transformation,o3d.pipelines.registration.TransformationEstimationPointToPlane())

print("Global TF is: \n",result_global.transformation)
print("ICP TF is: \n",result.transformation)

skeleton.transform(result.transformation)
o3d.visualization.draw_geometries([skeleton, downstl, pcd])