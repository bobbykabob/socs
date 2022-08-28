from math import pi


class net:
    def __init__(self, sim):
        self.sim = sim
        self.vertices, self.indices = self.sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/net.stl', 1, 0, 0.03)
        print(type(self.vertices))
        print(type(self.indices))
        self.shading_angle = 20.0 * 3.1415 / 180.0
        print(type(self.shading_angle))
        for i in range(len(self.vertices)):
            self.net_handle = self.sim.createMeshShape(0, self.shading_angle, self.vertices[i], self.indices[i])
        print(self.net_handle)

    def set_net_pos(self, pos):
        self.sim.setObjectPosition(self.net_handle, self.sim.handle_world, [pos[0], pos[1], 0.1])

    def set_net_orientation(self, beta):
        self.sim.setObjectOrientation(self.net_handle, self.sim.handle_world, [-pi / 2, beta, -pi / 2])

    def scale_net(self, y_scale, z_scale):
        self.sim.scaleObject(self.net_handle, 1, y_scale, z_scale, 0)
