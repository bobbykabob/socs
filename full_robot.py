import math

import numpy
import cv2
import matplotlib.pyplot as plt


class full_robot:


    def __init__(self, sim, name):
        self.vision_sensors = []
        self.name = name
        self.y = None
        self.x = None
        self.full_img = None
        self.sim = sim
        self.handle = self.sim.getObject(name)
        self.script_handle = self.sim.getScript(1, self.handle)
        self.sim.initScript(self.script_handle)
        for i in range(4):
            a_vision_sensor = self.sim.getObject(name + '/Vision_sensor[' + str(i) + ']')
            self.vision_sensors.append(a_vision_sensor)

    def generate_full_img(self):
        self.full_img = []
        for i in range(len(self.vision_sensors)):
            self.sim.handleVisionSensor(self.vision_sensors[i])
            img = self.sim.getVisionSensorDepth(self.vision_sensors[i], self.sim.handleflag_depthbuffer, [0, 0],
                                                [0, 0])

            img = img[0]

            img = self.sim.unpackFloatTable(img, 0, 65536, 0)
            img = numpy.array(img).reshape(1, 256, 1)

            img = cv2.flip(img, 0)
            self.full_img.append(img)

        circular_img = numpy.concatenate((self.full_img[2], self.full_img[3], self.full_img[1], self.full_img[0]),
                                         axis=1)
        self.full_img = circular_img

        return self.full_img

    def calculate_polar_coordinates(self):
        self.x = []
        self.y = []
        offset = -3 * numpy.pi / 4
        data_points = 4 * 256

        for i in range(128):
            distance = self.full_img[0, i * 8]
            # currently, 0 ranges from 0 - 1, but we need to scale into meters
            distance *= 2
            self.x.append(offset + 2 * numpy.pi * (data_points - i * 8) / data_points)
            self.y.append(distance)

        return self.x, self.y

    def init_local_graph(self):

        self.x = []
        self.y = []
        self.figure, self.ax = plt.subplots(figsize=(2.5, 4), subplot_kw={'projection': 'polar'})
        self.line1, = self.ax.plot(self.x, self.y, 'bo')
        r = numpy.arange(0, 2, 0.01)
        theta = numpy.pi / 2 + r * 0
        self.ax.plot(theta, r, color="green")
        # setting title
        title = "radial view: " + self.name
        plt.title(title)
        self.figure.canvas.set_window_title(title)

        # setting x-axis label and y-axis label
        plt.xlabel("X-axis")
        plt.ylabel("Y-axis")

    def update_local_graph(self):
        self.rotate_coordinates()
        self.line1.set_xdata(self.x)
        self.line1.set_ydata(self.y)
        self.figure.canvas.draw()
        plt.pause(0.000000000001)
        self.figure.canvas.flush_events()

    def move_robot(self, target_pos, constants):
        self.sim.callScriptFunction("update_actuation", self.script_handle, target_pos, constants)
    def show_depth_view(self):
        circular_img = self.full_img
        full_img = self.full_img
        for i in range(50):
            full_img = numpy.concatenate((full_img, circular_img), axis=0)
        cv2.imshow('depth view: ' + self.name, full_img)

    def rotate_coordinates(self):
        current_angles = self.sim.getObjectOrientation(self.handle, self.sim.handle_world)
        for i in range(len(self.x)):

            self.x[i] = self.x[i] + math.pi/2 + current_angles[2]

    def get_global_coordinates(self):
        global_x = []
        global_y = []

        # self.x = theta
        # self.y = r

        # we utilize the x = rcostheta & y = rsintheta to convert into local rectangular coordinates
        # we then call the position of our own robot, to convert into global rectangular coordinates
        current_pos = self.sim.getObjectPosition(self.handle, self.sim.handle_world)


        robot_x = current_pos[0]
        robot_y = current_pos[1]
        for i in range(len(self.x)):
            local_x = self.y[i] * math.cos(self.x[i])
            local_y = self.y[i] * math.sin(self.x[i])
            global_x.append(robot_x + local_x)
            global_y.append(robot_y + local_y)
        return global_x, global_y
