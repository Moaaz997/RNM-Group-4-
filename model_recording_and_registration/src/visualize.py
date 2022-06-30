import open3d as o3d
import numpy as np

# pcd=o3d.io.read_point_cloud("conversion_result.pcd")
# downpcd = pcd.voxel_down_sample(voxel_size=0.07)

# o3d.visualization.draw_geometries_with_editing([pcd])

pcd=o3d.io.read_point_cloud("cropped_2.ply")
downpcd = pcd.voxel_down_sample(voxel_size=0.005)

downpcd.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.7*2, max_nn=30))

pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        downpcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.07*5, max_nn=100))


stl=o3d.io.read_triangle_mesh("Skeleton_T.stl")
skeleton=stl.sample_points_uniformly(5000000)
downstl=skeleton.voxel_down_sample(voxel_size=0.005)
# downstl=o3d.geometry.VoxelGrid.create_from_point_cloud(skeleton,
#                                                               voxel_size=0.07)

distance_threshold=0.07*3


downstl.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.005*2, max_nn=30))

stl_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        downstl,
        o3d.geometry.KDTreeSearchParamHybrid(radius=0.005*5, max_nn=100))

result_global= o3d.pipelines.registration.registration_fast_based_on_feature_matching(
        downpcd, downstl, pcd_fpfh, stl_fpfh,
        o3d.pipelines.registration.FastGlobalRegistrationOption(
            use_absolute_scale=False))


result = o3d.pipelines.registration.registration_icp(
        downpcd, downstl, distance_threshold, result_global.transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPlane())

# result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
#     downpcd, downstl, pcd_fpfh, stl_fpfh, True,
#     distance_threshold,
#     o3d.pipelines.registration.TransformationEstimationPointToPoint(True),
#     3, [
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
#             0.9),
#         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
#             distance_threshold)
#     ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))

print("Global TF is: \n",result_global.transformation)
print("ICP TF is: \n",result.transformation)

downpcd.transform(result.transformation)
o3d.visualization.draw_geometries([ downstl, downpcd])
