import time
from math import pi

from random import randint

import numpy

from zmqRemoteApi import RemoteAPIClient
from constants import ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b
from math import radians
import cv2
# initial setup for client-sim
client = RemoteAPIClient()
sim = client.getObject('sim')
client.setStepping(True)

sim.startSimulation()

# empty arrays to cycle through
robots = []
vision_sensors = []
prev_time = int(round(time.time() * 1000))

name = '/robot'
robots.append(sim.getObject(name))
script_handle = sim.getScript(1, robots[0])
sim.initScript(script_handle)
vision_sensors.append(sim.getObject(name + '/Vision_sensor[0]'))
vision_sensors.append(sim.getObject(name + '/Vision_sensor[1]'))
vision_sensors.append(sim.getObject(name + '/Vision_sensor[2]'))
vision_sensors.append(sim.getObject(name + '/Vision_sensor[3]'))


for i in range(len(robots)):
    target_angle = radians(0)
    sim.callScriptFunction("update_actuation", script_handle, [-10, 10, target_angle],
                           [ROBOT_C1, ROBOT_C2, ROBOT_TRACK_WIDTH, b])

sim.setObjectPosition(robots[0], sim.handle_world, [0, 0, 0.1])
sim.setObjectOrientation(robots[0], sim.handle_world, [0, 0, pi/2])

while (t := sim.getSimulationTime()) < 40:
    full_img = []
    for i in range(len(vision_sensors)):
        sim.handleVisionSensor(vision_sensors[i])
        img, resX, resY = sim.getVisionSensorCharImage(vision_sensors[i])
        img = numpy.frombuffer(img, dtype=numpy.uint8).reshape(resY, resX, 3)

        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        img = cv2.flip(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), 0)
        full_img.append(img)

    circular_img = numpy.concatenate((full_img[2],full_img[3],full_img[1],full_img[0]),axis=1)
    lined_img = circular_img[130:160,:]

    cv2.imshow('', circular_img)
    cv2.imshow('lined', lined_img)
    cv2.waitKey(1)

    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
