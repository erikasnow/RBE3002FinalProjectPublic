#!/usr/bin/env python

import roslib
import rospy
from map_helper import *


class ExpandMap:
    def __init__(self):
        """
        set up node
        """

        rospy.init_node('expand_map', anonymous=True)

        # subscribes to /rawmap
        self.rawSubscriber = rospy.Subscriber('map', OccupancyGrid, self.handle_raw)

        # publishes expanded map
        self.mapPublisher = rospy.Publisher('costmap', OccupancyGrid, queue_size=1)

        # global variables
        self.raw_map = OccupancyGrid()

    def handle_raw(self, msg):
        """
        Updates the map from the Gmapping node
        :param msg: OccupancyGrid
        :return:
        """
        self.raw_map = msg

    def expand_map(self, x):
        """
        Expands the map walls so that the robot doesn't run into anything
        :param x: int
        :return: OccupancyGrid
        """
        map = self.raw_map
        map_data = list(map.data)
        count = 1

        # find all wall cells and make non-wall neighbors walls for x number of times
        while count < x:
            for index in range(len(map.data)):
                if map.data[index] > 50:  # if cell is a wall
                    percent = map.data[index]
                    # index = map.data.index(cell) # very wrong
                    # print("index: " + str(index))
                    neighbors = get_neighbors(index, map, True)  # get all valid neighbors
                    # print("neighbors: " + str(neighbors))

                    for n in neighbors:
                        map_data[n] = percent  # expand the wall into all free neighbors

            print "Type of map.data: " + str(type(map.data))
            print "Type of map_data: " + str(type(map_data))
            map.data = tuple(map_data)
            print "Type of map.data: " + str(type(map.data))
            print "Type of map_data: " + str(type(map_data))
            count = count + 1

        # publish the final expanded map

        self.mapPublisher.publish(map)

    def fill_gaps(self, map):
        """
        Fills in any inaccessible places in the map
        :return:
        """
        # TODO fill in inaccessible places


if __name__ == '__main__':
    print("make ExpandMap")
    e = ExpandMap()
    size = 3
    oldmap = e.raw_map
    e.expand_map(size)  # this is an arbitrary number for now
    # keep updating the expanded map

    while not rospy.is_shutdown():
        if oldmap != e.raw_map:  # if the map is updated
            e.expand_map(size)
            oldmap = e.raw_map