import time
import random
import numpy
from zmqRemoteApi import RemoteAPIClient

client = RemoteAPIClient()
sim = client.getObject('sim')

client.setStepping(True)

sim.startSimulation()
robots = []
robots_left_joint = []
robots_right_joint = []


def move_robot(index, left_power, right_power):
    sim.setJointTargetVelocity(robots_left_joint[index], left_power)
    sim.setJointTargetVelocity(robots_right_joint[index], right_power)

for i in range(0,3):
    name = '/robot[' + str(i) + ']'
    robots.append(sim.getObject(name))
    robots_left_joint.append(sim.getObject(name + '/leftjoint'))
    robots_right_joint.append(sim.getObject(name + '/rightjoint'))
while (t := sim.getSimulationTime()) < 20:
    for i in range(len(robots)):
        left_speed = -10
        right_speed = 10
        move_robot(i, left_speed, right_speed)
        positions = sim.getObjectPosition(robots[i], sim.handle_world)
        orientation = sim.getObjectOrientation(robots[i],sim.handle_world)
        orientation_degree = numpy.degrees(orientation)
        print("robot" + str(i) + ": ")
        print("x: " + str(positions[0]))
        print("y: " + str(positions[1]))
        print("z: " + str(positions[2]))
        print("theta: " + str(orientation_degree))
    s = f'Simulation time: {t:.2f} [s]'
    print(s)
    client.step()


sim.stopSimulation()
