import rospy
import numpy as np
import matplotlib.pyplot as plt
import cv2
from voronoi import generate_voronoi
# define grid_resol as 1m so we can match the initial maps


class MyBinaryMap:
    def __init__(self, grid, grid_resol):  # service to request binary map creation
        self.grid = grid
        self.grid_resol = int(grid_resol)
        # rospy.init_node("binary_service_node")
        # rospy.loginfo("service node for binary map creation initialized")
        # binary = rospy.Service("/binary_map", GetMap, self.binaryGrid)

    def occupancy_to_binary(self):
        """

        :param grid: occupancy grid with values between 0 and 1, where 0 represent free and 1 obstacle. -1 --> unknown
        :return:
        """
        binary_grid = np.copy(self.grid)
        binary_grid = binary_grid/100
        #binary_grid[binary_grid >= 0.5] = 1
        binary_grid[binary_grid < 0] = 1
        #binary_grid[binary_grid < 0.5] = 0
        return binary_grid


    def reduce_resolution(self, binary_full_grid):
        """
        
        :param grid: occupancy grid
        :param new_shape:
        :return:
        """
        new_shape = int(self.grid.shape[0]/self.grid_resol), int(self.grid.shape[1]/self.grid_resol)
        new_grid = np.copy(binary_full_grid)
        sh = new_shape[0], self.grid.shape[0]//new_shape[0], new_shape[1], self.grid.shape[1]//new_shape[1]
        new_grid = new_grid.reshape(sh).mean(-1).mean(1)
        new_grid[new_grid > 0.2] = 1
        new_grid[new_grid <= 0.2] = 0

        return np.rint(new_grid)

    def plotter(self, binary):
        """
        
        :param binary: binary map to save and plot
        :return:
        """
        #kernel to erode map
        #kernel = np.ones
        plt.imshow(binary, cmap='Greys',  interpolation='nearest')
        plt.savefig('Generated/binary_map.png')
        #binary_eroded = cv2.erode(binary)
        generate_voronoi(binary)
        np.savetxt("Generated/binary_map.csv", binary, delimiter=",")

    def upscale(self, binary, factor):
        new_shape = np.shape(binary)
        dimx, dimy = np.array(np.shape(binary)) * factor
        binary_big = cv2.resize(binary, dsize = (dimx,dimy), interpolation=cv2.INTER_CUBIC)
        plt.imshow(binary_big, cmap='Greys',  interpolation='nearest')
        plt.savefig('Generated/binary_map_upscaled.png')
        return binary_big

    def run(self):
        binary_map = self.occupancy_to_binary()
        binary_map = self.reduce_resolution(binary_map)
        self.plotter(binary_map)
        binary_big =self.upscale(binary_map, 4)
        generate_voronoi(binary_big)

    def grid_to_img(grid):
        """

        :param grid:
        :return:
        """
        img = np.array(grid * 255, dtype=np.uint8)
        return cv2.adaptiveThreshold(img, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 3, 0)


    def binaryGrid(self):
        
        dim = np.shape(self.grid)
        # print('shape of the grid is: ',)
        resol_x, resol_y = int(dim[0]/self.grid_resol), int(dim[1]/self.grid_resol)
        # print('resolution X is', resol_x, 'and resolution Y is', resol_y)
        binary_grid = np.zeros((resol_x,resol_y))
        for i in range(resol_x):
            for j in range(resol_y):
                
                copyMatrix = self.grid[i*self.grid_resol:i*self.grid_resol+self.grid_resol,j*self.grid_resol:j*self.grid_resol+self.grid_resol].copy()
                np.where(copyMatrix == -1, 100, copyMatrix) # it may be necessary to increase this value in case the laser info goes beyond walls
                # np.where(copyMatrix >= 50, 100, copyMatrix)
                # np.where(copyMatrix < 50, 0, copyMatrix)
                copyMatrix = copyMatrix/100
                binary_grid[int(i)][int(j)] = int(round(np.mean(copyMatrix, dtype=np.float64),0))
        # Generate image of the grid
        plt.imshow(binary_grid, cmap='Greys',  interpolation='nearest')
        plt.savefig('Generated/binary_map.png')
        # Generate CSV file with map
        np.savetxt("Generated/binary_map.csv", binary_grid, delimiter=",")
        return(binary_grid)

# for script testing
# if __name__ == "__main__":
    
    # resolution = 1
    # b = MyBinaryMap

#     grid = np.random.rand(500, 500)
#     print(grid.shape)
#     print(grid)

#     small = reduce_resolution(grid, (100, 100))
#     print(small.shape)
#     print(small)

#     binarized_grid = occupancy_to_binary(small)
#     print(binarized_grid.shape)
#     print(binarized_grid)

#     cv2.imshow("Map", grid_to_img(grid))
#     cv2.imshow("Map Grey", grid_to_img(small))
#     cv2.imshow("Map BW", grid_to_img(binarized_grid))
#     cv2.waitKey(0)
