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
    full_img = []

    for i in range(len(vision_sensors)):
        sim.handleVisionSensor(vision_sensors[i])
        img, resX, resY = sim.getVisionSensorCharImage(vision_sensors[i])
        img = sim.getVisionSensorDepth(vision_sensors[i], sim.handleflag_depthbuffer, [0, 0], [0, 0])
        res = img[1]
        img = img[0]
        lower_bound = (110) * 4 * 256
        upper_bound = (111) * 4 * 256

        img = img[lower_bound:upper_bound]
        img = sim.unpackFloatTable(img, 0, 65536, 0)
        resX = res[0]
        resY = res[1]
        print(len(img))
        #img = numpy.frombuffer(img, dtype=numpy.uint8).reshape(resY, resX, 4)
        #img = img[:,:,3]
        img = numpy.array(img).reshape(1, resY, 1)
        # In CoppeliaSim images are left to right (x-axis), and bottom to top (y-axis)
        # (consistent with the axes of vision sensors, pointing Z outwards, Y up)
        # and color format is RGB triplets, whereas OpenCV uses BGR:
        img = cv2.flip(img, 0)
        full_img.append(img)

    circular_img = numpy.concatenate((full_img[2],full_img[3],full_img[1],full_img[0]),axis=1)
    full_img = circular_img
    for i in range(50):
        full_img = numpy.concatenate((full_img, circular_img),axis=0)
    # full_img = cv2.bitwise_not(full_img)
    #lined_img = circular_img[130:160,:]

    cv2.imshow('depth view', full_img)
    #cv2.imshow('lined', lined_img)
    # creating new Y values
    theta = numpy.linspace( 0 , 2 * numpy.pi , 150 )
    x = []
    y = []
    max_distance = 1
    offset = -3*numpy.pi/4
    for i in range(256 * 4):
        distance = circular_img[0,i]


        x.append(offset + 2 * numpy.pi * (256 * 4 - i) / (256 * 4))
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
    cv2.waitKey(1)

    s = f'Simulation time: {t:.2f} [s]'
    current_time = int(round(time.time() * 1000))
    print('cycle time: ' + str(current_time - prev_time) + 'ms')
    prev_time = current_time
    print(s)
    client.step()

# end simulation
sim.stopSimulation()
