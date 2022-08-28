import time

from zmqRemoteApi import RemoteAPIClient
from net import net

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
    nets[n].set_net(n * 5, n * 5)
while (t := sim.getSimulationTime()) < 30:
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
