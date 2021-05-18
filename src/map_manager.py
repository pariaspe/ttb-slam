#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, SetMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import matplotlib.pyplot as plt

from map import MyMap


class MapManager:
    def __init__(self):
        rospy.init_node("MapManager")
        rospy.loginfo("Node Map manager initialized")
        
        self._map = MyMap()
        self.map_getter = rospy.Service("my_map/get", GetMap, self.get_occupancy_grid)
        self.map_setter = rospy.Service("my_map/set", SetMap, self.set_occupancy_grid)

        self.binary_srv = rospy.Service("my_map/binary/get", GetMap, self.get_binary_map)
 
        # binary resolution map (include in brain or get from ros param)
        self.binary_resol = int(1/0.05)

    def get_occupancy_grid(self, req):
        return self._map.to_msg()

    def set_occupancy_grid(self, req):
        self._map.from_msg(req.map)
        return True

    def get_binary_map(self, req):
        return self._map.binary_msg

    def occupancy_to_binary(self):
        # prepares map to filter to binary
        binary_grid = np.copy(self._map)
        binary_grid = binary_grid/100
        binary_grid[binary_grid < 0] = 1

        # reduces resolution of map to filter error
        new_shape = int(self._map.shape[0]/self.binary_resol), int(self._map.shape[1]/self.binary_resol)
        new_grid = np.copy(binary_grid)
        sh = new_shape[0], self._map.shape[0]//new_shape[0], new_shape[1], self._map.shape[1]//new_shape[1]
        new_grid = new_grid.reshape(sh).mean(-1).mean(1)
        new_grid[new_grid > 0.2] = 1
        new_grid[new_grid <= 0.2] = 0

        return np.rint(new_grid)

    def binary_plotter(self, binary_filtered):
        plt.imshow(binary_filtered, cmap='Greys',  interpolation='nearest')
        plt.savefig('Generated/binary_map.png')
        np.savetext("Generated/binary_map.csv", binary_filtered, delimiter=",")


if __name__ == "__main__":
    map_manager = MapManager()
    rospy.spin()
