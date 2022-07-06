
import open3d as o3d
import numpy as np

# pcd=o3d.io.read_point_cloud("conversion_result.pcd")
# downpcd = pcd.voxel_down_sample(voxel_size=0.07)

# o3d.visualization.draw_geometries_with_editing([pcd])
voxel_size=0.03
pcd=o3d.io.read_point_cloud("conversion_new_result.pcd")
# downpcd = pcd.voxel_down_sample(voxel_size=voxel_size)

# skeleton=o3d.io.read_point_cloud("/home/moaaz/catkin_ws/src/model_recording_and_registration/src/skeleton_1.ply")
# downstl=skeleton.voxel_down_sample(voxel_size=voxel_size)

# downpcd.estimate_normals(
#         o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))

# pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         downpcd,
#         o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))

# downstl.estimate_normals(
#         o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*2, max_nn=30))

# stl_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
#         downstl,
#         o3d.geometry.KDTreeSearchParamHybrid(radius=voxel_size*5, max_nn=100))


# result_global= o3d.pipelines.registration.registration_fast_based_on_feature_matching(
#         pcd, skeleton, pcd_fpfh, stl_fpfh,
#         o3d.pipelines.registration.FastGlobalRegistrationOption(
#             use_absolute_scale=False))

# distance_threshold=voxel_size*1.5


# # result_global = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
# #     downpcd, downstl, pcd_fpfh, stl_fpfh, True,
# #     distance_threshold,
# #     o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
# #     3, [
# #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
# #             0.9),
# #         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
# #             distance_threshold)
# #     ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))


# result = o3d.pipelines.registration.registration_icp(
#         downpcd, downstl, distance_threshold, result_global.transformation,
#         o3d.pipelines.registration.TransformationEstimationPointToPlane())


# print("Global TF is: \n",result_global.transformation)
# print("ICP TF is: \n",result.transformation)

# downpcd.transform(result.transformation)
o3d.visualization.draw_geometries_with_editing([ pcd])