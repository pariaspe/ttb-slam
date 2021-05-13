#!/usr/bin/env python
import rospy
from nav_msgs.srv import GetMap, SetMap
from nav_msgs.msg import OccupancyGrid


class MapManager:
    def __init__(self):
        rospy.init_node("MapManager")
        rospy.loginfo("Node initialized")

        self._map = OccupancyGrid()
        self.map_getter = rospy.Service("my_map/get", GetMap, self.get_occupancy_grid)
        self.map_setter = rospy.Service("my_map/set", SetMap, self.set_occupancy_grid)

        self.binary_srv = rospy.Service("my_map/binary/get", GetMap, self.get_binary_map)

    def get_occupancy_grid(self, req):
        return self._map

    def set_occupancy_grid(self, req):
        self._map = req.map
        return True

    def get_binary_map(self, req):
        pass


if __name__ == "__main__":
    l2pc = MapManager()
    rospy.spin()
