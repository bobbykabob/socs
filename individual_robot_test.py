import time

import move_robot
from constants import NUM_OF_ROBOTS
from zmqRemoteApi import RemoteAPIClient

# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
robots = []
robots_left_joint = []
robots_right_joint = []
prev_time = int(round(time.time() * 1000))


name = '/robot'
robots.append(sim.getObject(name))
robots_left_joint.append(sim.getObject(name + '/leftjoint'))
robots_right_joint.append(sim.getObject(name + '/rightjoint'))

# loops through simulation in seconds
while (t := sim.getSimulationTime()) < 50:
    positions = sim.getObjectPosition(robots[0], sim.handle_world)
    orientation = sim.getObjectOrientation(robots[0], sim.handle_world)
    left_power, right_power = move_robot.move_robot([0, 0, 0], [positions[0], positions[1], orientation[2]])
    sim.setJointTargetVelocity(robots_left_joint[0], left_power)
    sim.setJointTargetVelocity(robots_right_joint[0], right_power)

    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
