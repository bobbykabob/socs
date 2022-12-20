import math

import numpy
import cv2
import matplotlib.pyplot as plt


class full_robot:

    def __init__(self, sim, name):
        self.name = name

        self.sim = sim
        self.handle = self.sim.getObject(name)
        self.script_handle = self.sim.getScript(1, self.handle)
        self.sim.initScript(self.script_handle)

    def move_robot(self, target_pos, constants):
        self.sim.callScriptFunction("update_actuation", self.script_handle, target_pos, constants)

    def get_global_coordinates(self):
        global_x, global_y = self.sim.callScriptFunction("get_global_coordinates", self.script_handle)
        return global_x, global_y
    def get_global_position(self):
        x, y = self.sim.callScriptFunction("get_position", self.script_handle)
        return x,y