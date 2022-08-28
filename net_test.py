import time

from zmqRemoteApi import RemoteAPIClient
from net import net
import generate_new_position
from math import pi

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through


prev_time = int(round(time.time() * 1000))

# loops through simulation in seconds

nets = []
num_of_nets = 5
for n in range(num_of_nets):
    a_net = net(sim)
    nets.append(a_net)

for n in range(num_of_nets):
    new_position = generate_new_position.oval(n, 0, 0, 2, 2, 0)
    nets[n].set_net_pos(new_position)
    nets[n].set_net_orientation(pi - n * 2 * pi / num_of_nets)

while (t := sim.getSimulationTime()) < 50:


    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
