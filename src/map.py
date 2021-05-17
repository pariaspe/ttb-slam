from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import tf

import time
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import cv2
from planner import generate_voronoi


class MyMap:
    def __init__(self, grid=[],  resolution=0):
        self._grid = np.array(grid, ndmin=2)
        self._resolution = int(resolution)

        self._frame = "map"
        self._width, self._height = self._grid.shape

    @property
    def grid(self):
        return self._grid

    @property
    def binary(self):

        return self.occupancy_to_binary(self._grid)

    @property
    def binary_msg(self):
        binary_map = self.binary
        w, h = binary_map.shape
        return self._to_msg(self._frame, self._resolution, w, h, binary_map)

    def to_msg(self):
        return self._to_msg(self._frame, self._resolution, self._width, self._height, self._grid)

    @staticmethod
    def _to_msg(frame, resolution, width, height, grid):
        msg = OccupancyGrid()
        msg.header.frame_id = frame
        msg.info.map_load_time.secs = round(time.time())
        msg.info.resolution = resolution
        msg.info.width = width
        msg.info.height = height

        # Set the origin of the occupancy grid to fit the map
        grid_orientation = tf.transformations.quaternion_from_euler(pi, 0, pi / 2)
        msg.info.origin = Pose()
        msg.info.origin.orientation.x = grid_orientation[0]
        msg.info.origin.orientation.y = grid_orientation[1]
        msg.info.origin.orientation.z = grid_orientation[2]
        msg.info.origin.orientation.w = grid_orientation[3]
        msg.data = grid.ravel().tolist()

        return msg

    def from_msg(self, msg):
        self._resolution = msg.info.resolution
        self._width = msg.info.width
        self._height = msg.info.height
        self._grid = np.reshape(np.array(msg.data), [len(msg.data) / self._width, len(msg.data) / self._height])

    @staticmethod
    def occupancy_to_binary(grid):
        """2D Numpy array filled with 0 (free-space) and 1 (obstacles/unknown)"""
        binary_grid = np.copy(grid)
        print(binary_grid)
        binary_grid[binary_grid < 0] = 100
        binary_grid[binary_grid < 21] = 0
        binary_grid[binary_grid > 20] = 1
        # binary_grid = binary_grid / 100
        print(binary_grid)
        return binary_grid

    @staticmethod
    def downsample_grid(grid, resolution):
        """Reduces grid resolution to a given one"""
        new_shape = int(grid.shape[0] / resolution), int(grid.shape[1] / resolution)
        sh = new_shape[0], grid.shape[0]//new_shape[0], new_shape[1], grid.shape[1]//new_shape[1]
        return np.reshape(grid, sh).mean(-1).mean(1)

    @staticmethod
    def to_img(grid_):
        img = np.array(grid_ * 255, dtype=np.uint8)
        return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)

    def __repr__(self):
        str_map = ""
        for row in self.grid.tolist():
            for c in row:
                str_map += " " if int(c) == 0 else "X"
            str_map += "\n"
        return str_map

    def plotter(self, binary):
        """

        :param binary: binary map to save and plot
        :return:
        """
        # kernel to erode map
        # kernel = np.ones
        plt.imshow(binary, cmap='Greys', interpolation='nearest')
        plt.savefig('Generated/binary_map.png')
        # binary_eroded = cv2.erode(binary)
        generate_voronoi(binary)
        np.savetxt("Generated/binary_map.csv", binary, delimiter=",")

    def upscale(self, binary, factor):
        new_shape = np.shape(binary)
        dimx, dimy = np.array(np.shape(binary)) * factor
        binary_big = cv2.resize(binary, dsize=(dimx, dimy), interpolation=cv2.INTER_CUBIC)
        plt.imshow(binary_big, cmap='Greys', interpolation='nearest')
        plt.savefig('Generated/binary_map_upscaled.png')
        return binary_big

    def run(self):
        binary_map = self.occupancy_to_binary()
        binary_map = self.reduce_resolution(binary_map)
        self.plotter(binary_map)
        binary_big = self.upscale(binary_map, 4)
        generate_voronoi(binary_big)


# for script testing
if __name__ == "__main__":
    my_map = MyMap(grid=np.random.rand(1000, 1000)*100, resolution=1)
    # my_map = MyMap(grid=np.zeros(100), resolution=1)

    my_binary = MyMap()
    my_binary.from_msg(my_map.binary_msg)

    # print(my_map)
    # print("----------")
    # print(my_binary)

    cv2.imshow("Map", MyMap.to_img(my_map.grid))
    cv2.imshow("Binary", MyMap.to_img(my_binary.grid))
    cv2.imshow("Downsample", MyMap.to_img(my_binary.downsample_grid(my_binary.grid, int(1/0.05))))
    cv2.waitKey(0)
