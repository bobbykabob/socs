import time

from random import randint
from zmqRemoteApi import RemoteAPIClient
from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
robots = []

prev_time = int(round(time.time() * 1000))

name = '/robot'
robots.append(sim.getObject(name))
script_handle = sim.getScript(1, robots[0])
sim.initScript(script_handle)
# loops through simulation in seconds
for i in range(len(robots)):
    sim.callScriptFunction("update_actuation", script_handle, [0, 0, 0],
                           [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b])
randx = randint(1, 10)
randy = randint(1, 10)
sim.setObjectPosition(robots[0], sim.handle_world, [randx, randy, 1.0])
while (t := sim.getSimulationTime()) < 20:
    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
