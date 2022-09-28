from math import pi


class obstacle:
    def __init__(self, sim):
        self.sim = sim
        self.vertices, self.indices = self.sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/obstacle1.stl', 1, 0, 0.01)
        print(self.vertices)
        print(len(self.vertices[0]))
        print(self.indices)
        print(len(self.indices[0]))
        self.shading_angle = 20.0 * 3.1415 / 180.0
        print(type(self.shading_angle))
        for i in range(len(self.vertices)):
            self.obstacle_handle = self.sim.createMeshShape(0, self.shading_angle, self.vertices[i], self.indices[i])
        print(self.obstacle_handle)
        self.sim.setObjectInt32Param(self.obstacle_handle, 3004, 1)
        self.sim.setObjectInt32Param(self.obstacle_handle, 3024, 1)

    def set_obstacle_pos(self, pos):
        self.sim.setObjectPosition(self.obstacle_handle, self.sim.handle_world, [pos[0], pos[1], 0.1])

    def set_obstacle_orientation(self, beta):
        self.sim.setObjectOrientation(self.obstacle_handle, self.sim.handle_world, [-pi / 2, beta, -pi / 2])

    def scale_net(self, y_scale, z_scale):
        self.sim.scaleObject(self.obstacle_handle, 1, y_scale, z_scale, 0)
