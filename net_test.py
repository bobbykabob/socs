

import numpy
import numpy as np

from scipy.interpolate import CubicSpline
from shapely.geometry import LineString
import open3d as o3d

import generate_new_position
from constants import NUM_OF_ROBOTS

import time
from math import pi


from zmqRemoteApi import RemoteAPIClient
from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from math import radians
# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
robots = []
script_handle = []
robot_positions = []


for i in range(0, NUM_OF_ROBOTS):
    name = '/robot[' + str(i) + ']'

    robots.append(sim.getObject(name))

    script_handle.append(sim.getScript(1, robots[i]))
    sim.initScript(script_handle[i])


# looping through all robots
for i in range(len(robots)):
    new_position = generate_new_position.oval_opening(i, 0, 0, 2, 2, 0, pi/8)
    robot_positions.append(new_position)
    sim.callScriptFunction("update_actuation", script_handle[i], [new_position[0], new_position[1], 0],
                           [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b])

# loops through simulation in seconds

nets = []
num_of_nets = NUM_OF_ROBOTS
# net(sim)

x_position = []
y_position = []
u = []
for i in range(num_of_nets):
    new_position = generate_new_position.oval_opening(i, 0, 0, 2, 2, 0, pi/8)

    x_position.append(new_position[0])
    y_position.append(new_position[1])
    u.append(i)

xy_position = np.c_[x_position, y_position]

print("xy_position" + str(xy_position))
print("u" + str(u))
spline = CubicSpline(u, xy_position, extrapolate=False)

xs = (num_of_nets + 1) * np.linspace(0, 1, 500)
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

# generate a point cloud
vertices_3d = []

for n in range(int(len(patch_vertices_x))):
    current_index = n % len(patch_vertices_x)

    x_pos = patch_vertices_x[current_index]
    y_pos = patch_vertices_y[current_index]

    # bottom dimension of net
    z_pos = 0.004
    row = [x_pos, y_pos, z_pos]
    vertices_3d.append(row)

    # top dimension of net
    z_pos = 0.01
    row = [x_pos, y_pos, z_pos]
    vertices_3d.append(row)

# input point cloud into Open3D file types
vertices_3d = numpy.array(vertices_3d)
print(vertices_3d)
pcd = o3d.geometry.PointCloud()

# now we include details about the Point Cloud: we need the xyz position
pcd.points = o3d.utility.Vector3dVector(vertices_3d[:, :3])

# we utilize open3d's estimate normal method to estimate each individual point's normal plane
o3d.geometry.PointCloud.estimate_normals(
    pcd,
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
                                                      max_nn=30))

# to create a mesh, we use the Ball Pivoting Algorithm, or bpa, mesh-creating method
# this blog had a very good explanation:
# https://towardsdatascience.com/5-step-guide-to-generate-3d-meshes-from-point-clouds-with-python-36bad397d8ba

distances = pcd.compute_nearest_neighbor_distance()
avg_dist = np.mean(distances)
radius = 3 * avg_dist

bpa_mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, o3d.utility.DoubleVector(
    [radius, radius * 2]))

# we now post-process our mesh
dec_mesh = bpa_mesh

dec_mesh.remove_degenerate_triangles()
dec_mesh.remove_duplicated_triangles()
dec_mesh.remove_duplicated_vertices()
dec_mesh.remove_non_manifold_edges()

# we save our mesh into a stl for debugging purposes
output_path = 'C:/Users/harris/Desktop/CoppeliaSim/python/'
o3d.io.write_triangle_mesh(output_path + "p_mesh_c.stl", bpa_mesh)

# we now import the net and generate the net into coppeliasim
vertices, indices = sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/python/p_mesh_c.stl', 1, 0, 1)

shading_angle = 20.0 * 3.1415 / 180.0

for i in range(len(vertices)):
    net_handle = sim.createMeshShape(0, shading_angle, vertices[i], indices[i])
    # net_handle_2 = sim.createMeshShape(0, shading_angle, vertices[i], indices[i])
sim.setObjectPosition(net_handle, sim.handle_world, [0, 0, 0.05])

# we need to set the object to be respondable to other objects (like the sphere)
sim.setObjectInt32Param(net_handle, 3004, 1)
sim.setObjectInt32Param(net_handle, 3024, 1)

sim.setObjectInt32Param(net_handle, 3019, 61455)
sim.resetDynamicObject(net_handle)
# sim.setObjectInt32Param(net_handle_2,3004

prev_time = int(round(time.time() * 1000))

while (t := sim.getSimulationTime()) < 25:

    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()
for i in range(len(robots)):
    a_robot_position = robot_positions[i]
    a_robot_position[0] += 2
    sim.callScriptFunction("update_actuation", script_handle[i], [a_robot_position[0], a_robot_position[1], 0],
                           [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b])

while (t := sim.getSimulationTime()) < 100:
    x_average = 0
    y_average = 0
    for i in range(len(robots)):
        positions = sim.getObjectPosition(robots[i], sim.handle_world)
        x_average += positions[0]
        y_average += positions[1]
    x_average = x_average / len(robots)
    y_average = y_average / len(robots)

    sim.setObjectPosition(net_handle, sim.handle_world, [x_average, y_average, 0.05])
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()


# end simulation
sim.stopSimulation()
