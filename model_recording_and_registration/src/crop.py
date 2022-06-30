import open3d as o3d
import numpy as np

pcd=o3d.io.read_point_cloud("conversion.pcd")
downpcd = pcd.voxel_down_sample(voxel_size=0.07)

o3d.visualization.draw_geometries_with_editing([pcd])