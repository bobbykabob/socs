class net:
    def __init__(self, sim):
        self.sim = sim
        self.net_handle = self.sim.importMesh(0, '/Users/harris/Desktop/CoppeliaSim/net.simmodel.xml', 0, 0, 0.05)
        print(self.net_handle)

    def set_net(self, x: float, y: float):
        self.sim.setObjectPosition(self.net_handle, self.sim.handle_world, [x, y, 0.25])
