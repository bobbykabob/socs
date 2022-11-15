import time
from math import pi


import numpy
import matplotlib.pyplot as plt
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


plt.ion()

x = []
y = []
figure, ax = plt.subplots(figsize=(10, 10), subplot_kw={'projection': 'polar'})
line1, = ax.plot(x, y, 'bo')


r = numpy.arange(0, 2, 0.01)
theta = numpy.pi / 2 + r * 0
line2, = ax.plot(theta,r, color="green")
# setting title
plt.title("radial view", fontsize=20)

# setting x-axis label and y-axis label
plt.xlabel("X-axis")
plt.ylabel("Y-axis")

while (t := sim.getSimulationTime()) < 40:
    # create an empty area to store the image matrices
    full_img = []

    for i in range(len(vision_sensors)):
        # handles the Vision Sensor to update (clearing previous image)
        sim.handleVisionSensor(vision_sensors[i])

        # gets Vision sensor depth informaton
        img = sim.getVisionSensorDepth(vision_sensors[i], sim.handleflag_depthbuffer, [0, 0], [0, 0])

        # we select the image matrix of the returned value (img[0] is the matrix, img[1] is the resolution matrix)
        img = img[0]

        # we convert the int32 into int8 format
        img = sim.unpackFloatTable(img, 0, 65536, 0)

        # reshape into numpy & opencv format
        img = numpy.array(img).reshape(1, 256, 1)

        # flip the camera
        img = cv2.flip(img, 0)
        # append onto the circular image
        full_img.append(img)

    # concatenate into one matrix; previously it was an array of matirices
    circular_img = numpy.concatenate((full_img[2],full_img[3],full_img[1],full_img[0]),axis=1)
    full_img = circular_img

    # show the result
    cv2.imshow('depth view', full_img)

    # perform calculations with the acquired data; we create a polar graph from the data
    # x represents the theta value
    x = []
    # y represents the radius value
    y = []
    max_distance = 1
    offset = -3*numpy.pi/4

    data_points = 4 * 256
    for i in range(128):
        distance = circular_img[0, i*8]
        x.append(offset + 2 * numpy.pi * (data_points - i*8) / data_points)
        y.append(distance)

    # updating data values
    line1.set_xdata(x)
    line1.set_ydata(y)
    # drawing updated values
    figure.canvas.draw()

    # This will run the GUI event
    # loop until all UI events
    # currently waiting have been processed
    figure.canvas.flush_events()

    # process the user information - cycle time & simulation times
    s = f'Simulation time: {t:.3f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
