import matplotlib.pyplot as plt
import numpy as np
import math
from kernel.other_uavs.target_drone_trajectory import TargetDroneTrajectory
"""


this class is outdated!!! fix before using

"""
class PlotTrajectory:
    def __init__(self):

        self.listx = []
        self.listy = []
        self.listz = []
        self.listu = []
        self.listv = []
        self.listw = []

        self.path_model = TargetDroneTrajectory()


    def _trajectory(self):
        """ this function is suppose to define a list of trajectory points """

        while self.path_model.t < 1000: 
            self.path_model.lawnmower_trajectory()
            self.listx.append(self.path_model.x)
            self.listy.append(self.path_model.y)
            self.listz.append(self.path_model.z)
            self.listu.append(0)
            self.listv.append(0)
            self.listw.append(0)

    def plot (self):
        self._trajectory()

        # 3d figure setup
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self.listx, self.listy, self.listz)


        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title("3D Path with Direction")

        ax.set_xlim(58, 200)
        ax.set_ylim(20, 70)
        ax.set_zlim(-5, 5)

        ax.set_box_aspect([1, 1, 1])   # makes x,y,z scale look equal

        plt.show()


if __name__ == "__main__":
    PlotTrajectory().plot()