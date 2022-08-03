import open3d as o3d

import numpy as np
from matplotlib import pyplot as plt
from numpy.linalg import inv
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.patches import FancyArrowPatch
from mpl_toolkits.mplot3d import proj3d

transform_info = np.array([[-0.279217,  -0.641566,   0.714431,   0.359994],
                                          [-0.958189,   0.234405,  -0.163991, -0.0817953],
                                          [-0.0622546,  -0.730351,  -0.680221,   0.496731],
                                          [0,          0,          0,          1]])

def get_rot_trans_matrix(transform_info):
    rot_matrix = [[0, 0, 0], [0, 0, 0], [0, 0, 0]]
    for i in range(0, 3):
        for j in range(0, 3):
            rot_matrix[i][j] = transform_info[i][j]
    
    translate_matrix = [transform_info[0, 3], transform_info[1, 3], transform_info[2, 3]]
    return (np.asarray(rot_matrix), np.asarray(translate_matrix))

def get_slice_array(l, start, end):
    result = []
    for i in range(start, end):
        result.append(l[i])
    return result


extracted_info = get_rot_trans_matrix(transform_info)
rot_matrix = np.array(extracted_info[0])
print(rot_matrix)
rot_matrix_inverse = inv(rot_matrix)
translate_matrix = np.array(extracted_info[1])

fig = plt.figure()
ax = fig.add_subplot(projection='3d')

origin = np.array([0, 0, 0])
x_unit_base = np.array([1, 0, 0])
y_unit_base = np.array([0, 1, 0])
z_unit_base = np.array([0, 0, 1])

x_unit_cam = np.matmul(rot_matrix, x_unit_base)
y_unit_cam = np.matmul(rot_matrix, y_unit_base)
z_unit_cam = np.matmul(rot_matrix, z_unit_base)

pcd  = o3d.io.read_point_cloud('cheezItBB.ply')
# pcd = pcd.voxel_down_sample(voxel_size=0.005)
# pcd = pcd.voxel_down_sample(voxel_size=0.01)
# o3d.visualization.draw_geometries([pcd])
pcd_data = np.array(pcd.points)

for i in range(0, len(pcd_data)):
    pcd_data[i] = pcd_data[i] * 1/1000
    pcd_data[i] = np.add(pcd_data[i], translate_matrix)
    pcd_data[i] = np.matmul(rot_matrix, pcd_data[i])

# pcd = o3d.geometry.PointCloud()
# pcd.points = o3d.utility.Vector3dVector(pcd_data.astype(np.float64))

ax.scatter(pcd_data[0:, 0], pcd_data[0:, 1], pcd_data[0:, 2])

ax.quiver(origin[0], origin[1], origin[2], 0.25*x_unit_base[0], 0.25*x_unit_base[1], 0.25*x_unit_base[2], color='r', arrow_length_ratio=0.25)
ax.quiver(origin[0], origin[1], origin[2], 0.25*y_unit_base[0], 0.25*y_unit_base[1], 0.25*y_unit_base[2], color='g', arrow_length_ratio=0.25)
ax.quiver(origin[0], origin[1], origin[2], 0.25*z_unit_base[0], 0.25*z_unit_base[1], 0.25*z_unit_base[2], color='b', arrow_length_ratio=0.25)


ax.quiver(translate_matrix[0], translate_matrix[1], translate_matrix[2], 0.25*x_unit_cam[0], 0.25*x_unit_cam[1], 0.25*x_unit_cam[2], color='r', arrow_length_ratio=0.25)
ax.quiver(translate_matrix[0], translate_matrix[1], translate_matrix[2], 0.25*y_unit_cam[0], 0.25*y_unit_cam[1], 0.25*y_unit_cam[2], color='g', arrow_length_ratio=0.25)
ax.quiver(translate_matrix[0], translate_matrix[1], translate_matrix[2], 0.25*z_unit_cam[0], 0.25*z_unit_cam[1], 0.25*z_unit_cam[2], color='b', arrow_length_ratio=0.25)

plt.show()