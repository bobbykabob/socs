import time
from math import pi

import numpy
import numpy as np
import open3d.cpu.pybind.geometry
from scipy.interpolate import CubicSpline
from shapely.geometry import LineString
import open3d as o3d

import generate_new_position
from constants import NUM_OF_ROBOTS
from figures import BLUE
from net import net
from zmqRemoteApi import RemoteAPIClient

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through


prev_time = int(round(time.time() * 1000))

# loops through simulation in seconds

nets = []
num_of_nets = NUM_OF_ROBOTS
# net(sim)

x_position = []
y_position = []
u = []
for n in range(num_of_nets):
    new_position = generate_new_position.oval_opening(n, 0, 0, 0.3, 0.3, -pi / 3, 1 * pi / 8)

    x_position.append(new_position[0])
    y_position.append(new_position[1])
    u.append(n)

xy_position = np.c_[x_position, y_position]

print("xy_position" + str(xy_position))
print("u" + str(u))
spline = CubicSpline(u, xy_position, extrapolate=False)

xs = (num_of_nets + 1) * np.linspace(0, 1, 5000)
spline_x_position = spline(xs)[:, 0]
spline_y_position = spline(xs)[:, 1]

# convert from a scipy-specific array into a shapely-specific array
line = []
for n in range(len(spline_x_position)):
    line.append((spline_x_position[n], spline_y_position[n]))

# shapely buffer code

line = LineString(line)
dilated = line.buffer(0.001, resolution=1, cap_style=1, join_style=2)

# print vertices
patch_vertices_x, patch_vertices_y = dilated.exterior.coords.xy

vertices_3d = []

for n in range(int(len(patch_vertices_x))):
    current_index = n % len(patch_vertices_x)

    x_pos = patch_vertices_x[current_index]
    y_pos = patch_vertices_y[current_index]

    z_pos = 0.004
    row = [x_pos, y_pos, z_pos, 8, 8, 8]
    vertices_3d.append(row)
    z_pos = 0.008
    row = [x_pos, y_pos, z_pos, 8, 8, 8]
    vertices_3d.append(row)

vertices_3d = numpy.array(vertices_3d)
print(vertices_3d)
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(vertices_3d[:, :3])
pcd.colors = o3d.utility.Vector3dVector(vertices_3d[:, 3:6] / 255)
o3d.geometry.PointCloud.estimate_normals(
    pcd,
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                      max_nn=30))
if pcd.has_normals():
    print("has normal")

# pcd.normals = o3d.utility.Vector3dVector(vertices_3d[:, 6:9])
# o3d.visualization.draw_geometries([pcd])

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(
    [radius, radius * 2]))
dec_mesh = bpa_mesh

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

output_path = 'C:/Users/harris/Desktop/CoppeliaSim/python/'
o3d.io.write_triangle_mesh(output_path + "p_mesh_c.stl", bpa_mesh)

vertices, indices = sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/python/p_mesh_c.stl', 1, 0, 1)

shading_angle = 20.0 * 3.1415 / 180.0

for i in range(len(vertices)):
    net_handle = sim.createMeshShape(0, shading_angle, vertices[i], indices[i])
    net_handle_2 = sim.createMeshShape(0, shading_angle, vertices[i], indices[i])
sim.setObjectPosition(net_handle, sim.handle_world, [0, 0, 0.03])
sim.setObjectPosition(net_handle_2, sim.handle_world, [0, 0, 0.08])


sim.setObjectInt32Param(net_handle,3004,1)
sim.setObjectInt32Param(net_handle_2,3004,1)
while (t := sim.getSimulationTime()) < 500:
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
