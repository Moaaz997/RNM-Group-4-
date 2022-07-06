import open3d as o3d
import numpy as np

# pcd=o3d.io.read_point_cloud("conversion_result.pcd")


pcd=o3d.io.read_point_cloud("cropped_5.ply")

stl=o3d.io.read_triangle_mesh("Skeleton_T.stl")
skeleton=stl.sample_points_uniformly(len(pcd.points))
o3d.visualization.draw_geometries_with_editing([skeleton])
