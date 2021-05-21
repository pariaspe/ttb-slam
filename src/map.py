from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import tf

import time
from math import pi
import numpy as np
import matplotlib.pyplot as plt
import cv2


def generate_voronoi(original_img):
    """
    Loads a binary map and returns the voronoi representation
    :param original_img:
    :return:
    """

    # Load map as an image
    ret, original_img = cv2.threshold(original_img, 0, 1, cv2.THRESH_BINARY_INV)

    # Resize the image to improve voronoi precision
    mult = 10
    dim = (original_img.shape[1] * mult, original_img.shape[0] * mult)
    original_img = cv2.resize(original_img, dim, interpolation = cv2.INTER_AREA)

    img = original_img.copy()

    size = np.size(img)
    skel = np.zeros(img.shape, img.dtype)

    # element = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(15,15))  # Element for morph transformations
    # img = cv2.erode(img, element)

    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))  # Element for morph transformations
    done = False

    # Skelitization
    while not done:
        eroded = cv2.erode(img, element)
        temp = cv2.dilate(eroded, element)
        temp = cv2.subtract(img, temp)
        skel = cv2.bitwise_or(skel, temp)
        img = eroded.copy()

        # Stop when the image is fully eroded
        zeros = size - cv2.countNonZero(img)
        if zeros == size:
            done = True

    # Image showing

    # cv2.imshow("skel",skel)
    # cv2.imshow("image", original_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # Free spaces = 0
    ret, final_img = cv2.threshold(skel, 0, 1, cv2.THRESH_BINARY_INV)
    # plt.imshow(final_img, cmap='Greys', interpolation='nearest')
    # plt.savefig('Generated/voronoi.png')
    return final_img


class MyMap:
    def __init__(self, grid=[],  resolution=0):
        self._grid = np.array(grid, ndmin=2)
        self._resolution = resolution

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
        self._grid = np.reshape(np.array(msg.data), [int(len(msg.data) / self._width), int(len(msg.data) / self._height)])
        return self._grid

    @staticmethod
    def occupancy_to_binary(grid):
        """2D Numpy array filled with 0 (free-space) and 1 (obstacles/unknown)"""
        binary_grid = np.copy(grid)
        binary_grid[binary_grid < 0] = 100
        binary_grid[binary_grid < 0.21] = 0
        binary_grid[binary_grid > 0.20] = 1
        # binary_grid = binary_grid / 100
        print('binary_grid is returned')
        return binary_grid

    @staticmethod
    def binary_to_occupancy(binary):
        grid = np.copy(binary)
        grid = grid.astype(np.int) * 100
        return grid

    @staticmethod    
    def reduce_resolution(grid, resolution, binary_full_grid):
        """
            
        :param grid: occupancy grid
        :param binary_full_grid: binary grid without downscaling
        :return:
        """
        # multiply because this is grid resolution not binary map resolution as intended in original script
        new_shape = int(grid.shape[0]*resolution), int(grid.shape[1]*resolution)
        new_grid = np.copy(binary_full_grid)
        sh = new_shape[0], grid.shape[0]//new_shape[0], new_shape[1], grid.shape[1]//new_shape[1]
        new_grid = new_grid.reshape(sh).mean(-1).mean(1)
        new_grid[new_grid > 0.5] = 1
        new_grid[new_grid <= 0.5] = 0
        print('binary_grid resolution is reduced')
        return np.rint(new_grid)

    @staticmethod
    def downscale(grid, factor):
        """Reduces grid by a given factor"""
        new_shape = int(grid.shape[0] / factor), int(grid.shape[1] / factor)
        sh = new_shape[0], grid.shape[0]//new_shape[0], new_shape[1], grid.shape[1]//new_shape[1]
        return np.reshape(grid, sh).mean(-1).mean(1)

    def to_img(self, inverted=True):
        return self.grid_to_img(self.grid, inverted)

    @staticmethod
    def grid_to_img(grid_, inverted=True):
        if inverted:
            img = np.array(255 - grid_ * 255, dtype=np.uint8).astype('uint8')
        else:
            img = np.array(grid_ * 255, dtype=np.uint8).astype('uint8')
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

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

        np.savetxt("Generated/binary_map.csv", binary, delimiter=",")
        plt.imshow(binary, cmap='Greys', interpolation='nearest')
        plt.pause(3)
        plt.savefig('Generated/binary_map.png')
        print('binary_grid has been generated')
        # binary_eroded = cv2.erode(binary)
        #generate_voronoi(binary)
        #print('voronoi has been generated in plotter')

    @staticmethod
    def upscale(binary, factor):
        dimx, dimy = np.array(np.shape(binary)) * factor
        binary_big = cv2.resize(binary, dsize=(dimx, dimy), interpolation=cv2.INTER_CUBIC)
        # plt.imshow(binary_big, cmap='Greys', interpolation='nearest')
        # plt.savefig('Generated/binary_map_upscaled.png')
        return binary_big

    @staticmethod
    def expand(grid, factor):
        return np.kron(grid, np.ones((factor, factor)))

    def run(self):
        binary_map = self.occupancy_to_binary(self._grid)
        binary_map = self.reduce_resolution(self._grid, self._resolution, binary_map) #esta funcion no existe, la incluyo, downsample no se si se usa
        self.plotter(binary_map)
        binary_big = self.upscale(binary_map, 4)
        voronoi_map = generate_voronoi(binary_big)
        print('voronoi has been generated in run')
        return voronoi_map

    def mark_as_free(self, x, y):
        # correction for wall error
        x = x + 1
        y = y + 1
        if self._grid[x, y] == -1:  # unknown --> visited
            self._grid[x, y] = 20
        elif 50 <= self._grid[x, y] < 100:  # not 100% sure --> subtract 30
            self._grid[x, y] -= 20
        elif 10 <= self._grid[x, y] < 50:
            self._grid[x, y] -= 10
        elif self._grid[x, y] < 10:
            self._grid[x, y] = 0

    def mark_as_occupied(self, x, y):
        # correction for wall error
        x = x + 1
        y = y + 1
        # mark occupied cell
        if 50 < self._grid[x, y] <= 90:
            self._grid[x, y] += 10
        elif 50 >= self._grid[x, y]:
            self._grid[x, y] += 20
        elif self._grid[x, y] > 90:
            self._grid[x, y] = 100
        self._mark_as_probable_obs(x-1, y)
        self._mark_as_probable_obs(x, y-1)
        self._mark_as_probable_obs(x+1, y)
        self._mark_as_probable_obs(x, y+1)

    def _mark_as_probable_obs(self, x, y):
        if self._grid[int(x), int(y)] < 1:
            self._grid[int(x), int(y)] = 50
        elif self._grid[int(x), int(y)] < 90:
            self._grid[int(x), int(y)] += 20
        if self._grid[int(x), int(y)] > 100:
            self._grid[int(x), int(y)] = 100


# for script testing
if __name__ == "__main__":
    my_map = MyMap(grid=np.random.rand(1000, 1000)*100, resolution=1)
    # my_map = MyMap(grid=np.zeros(100), resolution=1)

    img = np.array([[1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 1., 1., 1., 1., 0., 0., 0., 1., 1., 1.],
                    [1., 1., 1., 1., 1., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 0., 0., 0., 0., 0., 0., 0., 1., 1., 1.],
                    [1., 1., 1., 1., 1., 1., 1., 1., 1., 1., 1.]])


    my_map = MyMap(grid=img, resolution=1)
    # my_binary = MyMap(grid=my_map.binary, resolution=1)
    my_down = MyMap(grid=my_map.downscale(my_map.grid, 1), resolution=1)
    # rescaled = my_down.upscale(my_down.binary, 4)

    # print(my_map)
    # print("----------")
    # print(my_binary)

    # plt.imshow(my_map.to_img(), cmap='Greys', interpolation='nearest')
    # plt.imshow(my_binary.to_img(), cmap='Greys', interpolation='nearest')
    # plt.imshow(MyMap.grid_to_img(rescaled), cmap='Greys', interpolation='nearest')
    # plt.pause(0)

    # cv2.imshow("Map", my_map.to_img())
    # cv2.imshow("Factor", MyMap.grid_to_img(rescaled))
    cv2.imshow("Voronoi", generate_voronoi(my_down.binary))
    cv2.waitKey(0)
