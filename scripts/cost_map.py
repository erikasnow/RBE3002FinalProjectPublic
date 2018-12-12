#!/usr/bin/env python
import rospy
from map_helper import *
from nav_msgs.msg import Path, GridCells
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped, Point
from math import sqrt, pow
from PriorityQueue import *
import std_msgs
import map_helper

class cost_map:


    def __init__(self):
        rospy.init_node("cost_map")

        self.gmap_subscriber = rospy.Subscriber('map', OccupancyGrid, self.expand_map)
        self.costmap_publisher = rospy.Publisher('costmap', OccupancyGrid, queue_size=10)

        self.old_map = None
        self.new_map = OccupancyGrid()


    def expand_map(self, msg):
        """

        :return:
        """
        self.old_map = msg
        cells = self.old_map.data
        new_data = list(cells)
        self.new_map = self.old_map

        for index in range(len(cells)):
            neighbors = []
            searched = []

            if cells[index] == -1:
                new_data[index] = -1
            if cells[index] == 100:
                neighbors.append(index)
                cost = cells[index]
                print"Found an Obstacle at index: " + str(index)
                while cost > 0:
                    curr_index = neighbors.pop(0)
                    if curr_index not in searched:
                        searched.append(curr_index)
                        neighbors.extend(get_neighbors(curr_index, self.old_map,True))

                        cost = self.calculate_weight(index, curr_index)
                        #print "calculated cost at " + str(curr_index) + " is " + str(cost)
                        if cost > self.new_map.data[curr_index] and cells[curr_index] != -1:
                            #print  "adding new cost " + str(cost) + " at index " + str(curr_index)
                            new_data[curr_index] = cost

        new_data = tuple(new_data)
        self.new_map.data = new_data
        self.costmap_publisher.publish(self.new_map)


    def calculate_weight(self, start, curr):
        """
        Calculates the move cost for the current cell based on the euclidean distance between the two
        :param start:
        :param curr:
        :return:
        """
        width = self.old_map.info.width

        curr_x = curr % width
        curr_y = (curr - curr_x)/width
        start_x = start % width
        start_y = (start - start_x)/width

        dif_x = start_x - curr_x
        dif_y = start_y - curr_y

        dist = sqrt(pow(dif_x,2) + pow(dif_y,2))

        cost = int(100 - pow(dist,3) + 8)

        if cost < 0:
            cost =  0
        if cost > 100:
            cost = 100

        return cost


if __name__== '__main__':
    cmap = cost_map()
    rospy.sleep(1)
    print "Initialized Cost Map"
    rospy.spin()
