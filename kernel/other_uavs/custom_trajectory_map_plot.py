import matplotlib.pyplot as plt
import numpy as np
import math
from kernel.other_uavs.target_drone_trajectory_custom import TargetDroneTrajectory

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
        while self.path_model.t < 10000:
            self.path_model.quintic_trajectory(40)

            self.listx.append(self.path_model.x)
            self.listy.append(self.path_model.y)
            self.listz.append(self.path_model.z)

            self.listu.append(0)
            self.listv.append(0)
            self.listw.append(0)

    def plot(self):
        self._trajectory()

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self.listx, self.listy, self.listz, label="trajectory")

        # start point
        ax.scatter(
            self.listx[0], self.listy[0], self.listz[0],
            s=80, marker="o", label="start"
        )

        # end point
        ax.scatter(
            self.listx[-1], self.listy[-1], self.listz[-1],
            s=80, marker="x", label="end"
        )

        # arrows along path
        step = 150
        for i in range(0, len(self.listx) - step, step):
            ax.quiver(
                self.listx[i],
                self.listy[i],
                self.listz[i],
                self.listx[i + step] - self.listx[i],
                self.listy[i + step] - self.listy[i],
                self.listz[i + step] - self.listz[i],
                length=1,
                normalize=True
            )

        ax.set_xlabel("x")
        ax.set_ylabel("y")
        ax.set_zlabel("z")
        ax.set_title("3D Path with Direction")

        ax.set_xlim(55, 125)
        ax.set_ylim(-5, 45)
        ax.set_zlim(-18, -2)

        ax.set_box_aspect([70, 50, 16])

        ax.legend()
        plt.show()


if __name__ == "__main__":
    PlotTrajectory().plot()