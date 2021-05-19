#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, SetMap, LoadMap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os

from map import MyMap


class MapManager:
    PUB_RATE = 100000000  # 0.1 sec

    def __init__(self):
        rospy.init_node("MapManager")
        rospy.loginfo("Node Map manager initialized")
        
        self._map = MyMap()
        self.map_getter = rospy.Service("my_map/get", GetMap, self.get_occupancy_grid)
        self.map_setter = rospy.Service("my_map/set", SetMap, self.set_occupancy_grid)
        self.is_map_loaded = False
        self.map_loader = rospy.Service("my_map/load", LoadMap, self.load_occupancy_grid)

        self.binary_srv = rospy.Service("my_map/binary/get", GetMap, self.get_binary_map)

        self.occup_grid_pub = rospy.Publisher("/my_map", OccupancyGrid, queue_size=1)
        self.grid_timer = rospy.Timer(rospy.Duration(nsecs=self.PUB_RATE), self.send_occupancy_grid)

    def get_occupancy_grid(self, req):
        return self._map.to_msg()

    def set_occupancy_grid(self, req):
        if not self.is_map_loaded:
            self._map.from_msg(req.map)
            return True
        else:
            rospy.logwarn_once("Map not set. Already loaded one.")
            return False

    def load_occupancy_grid(self, req):
        url = req.map_url

        rel_path = rospy.get_param('output_path', os.getcwd())
        url = os.path.join(rel_path, url)

        # Load explored map
        try:
            explored_map = np.genfromtxt(url, delimiter=',')
            self.is_map_loaded = True
        except ValueError:
            print("There is no map to load")
            return OccupancyGrid(), 1

        expanded = MyMap.expand(explored_map, 20)
        self._map = MyMap(MyMap.binary_to_occupancy(expanded), resolution=0.05)
        return self._map.to_msg(), 0

    def get_binary_map(self, req):
        return self._map.binary_msg

    def send_occupancy_grid(self, event):
        """Publish OccupancyGrid map"""
        self.occup_grid_pub.publish(self._map.to_msg())


if __name__ == "__main__":
    map_manager = MapManager()
    rospy.spin()
