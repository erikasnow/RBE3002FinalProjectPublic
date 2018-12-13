#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, GridCells
from map_helper import *


class Frontier:
    def __init__(self):
        """
            set up publishers, subscribers, and state-storing variables
        """
        rospy.init_node('find_frontiers', anonymous=True)

        # subscribes to /map and robot odometry
        self.mapSubscriber = rospy.Subscriber('costmap', OccupancyGrid, self.handle_map)
        self.odomSubscriber = rospy.Subscriber('odom', Odometry, self.handle_odom)

        # publishes nearest frontier and a list of all frontiers
        self.nearestPublisher = rospy.Publisher('nearest_frontier', PoseStamped, queue_size=1)
        self.frontierCellsPublisher = rospy.Publisher('unmapped_frontiers', GridCells, queue_size=10)

        # global variables
        self.mymap = OccupancyGrid()
        self.px = 0
        self.py = 0


    def handle_map(self, msg):
        """
            Receives map updates from the expand_map node
            :param msg: OccupancyGrid
            :return:
        """
        self.mymap = msg
        frontiers = self.find_all_frontiers()
        self.publish_frontiers(frontiers)
        self.find_next_target(frontiers)


    def handle_odom(self, msg):
        """
            Updates the current location of the robot
            :param msg: Odom
            :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y


    def find_all_frontiers(self):
        """
            Finds all frontiers in the map
            frontiers are known open cells (cells that aren't -1 or 100) that are:
             - next to unknown cells (-1 cells)
             - at least two cells away from the nearest wall
            :return: array of points containing all frontier cells
        """
        frontier_list = []

        # iterate through the whole map
        for cell_index in range(len(self.mymap.data)):

            close_to_wall = False
            next_to_unknown = False

            # if the cell is open, see if it is a frontier
            cell_value = self.mymap.data[cell_index]
            if cell_value is not -1 and cell_value is not 100:

                # an open cell is a frontier if the neighbors are correct,
                # so iterate through the neighbors of the cell
                for neighbor_index in get_neighbors(cell_index, self.mymap, True):

                    # check if the neighbor is unknown
                    if self.mymap.data[neighbor_index] is -1:
                        next_to_unknown = True

                    # check the neighbors of the neighbor to find walls
                    # within 2 cells of the robot
                    # we don't want the robot going too close to a known wall
                    for nni in get_neighbors(neighbor_index, self.mymap, True):
                        if self.mymap.data[nni] is 100:
                            close_to_wall = True
                            break

                    # if we already know the cell is close to a wall,
                    # skip checking the rest of the neighbors
                    if close_to_wall:
                        break

                # if the cell is a frontier, add it to the frontier list
                if next_to_unknown and not close_to_wall:
                    frontier_list.append(cell_index)

        return frontier_list


    def find_next_target(self, frontier_list):
        """
            run a breadth-first search from the robot's location until a
            cell from the list of frontiers is found
            :param frontier_list: an array of Point with all the good frontiers
            :return: a Point of the best fronier to travel to next
        """
        # TODO: this should actually find the first frontier cell that is a
        # certain distance away from the robot, so we can avoid getting stuck

        print "find_frontier is looking for the next target"

        # find the robot's current index to search from
        location = (self.px, self.py)
        start_index = point_to_index(location, self.mymap)

        # run a breadth-first search until we find a frontier cell
        queue = [start_index]
        visited = []
        while queue:
            current = queue.pop(0)
            visited.append(current)

            # if we've found a frontier cell, return it as a point
            if current in frontier_list:
                print "next target index is:\n" + str(current)
                return current

            # if we haven't found a frontier cell, add all the neighbors
            # that aren't walls to the queue
            else:
                for neighbor in get_neighbors(current, self.mymap, True):
                    if(self.mymap.data[neighbor] is not 100 and
                            neighbor not in visited):
                        queue.append(neighbor)

        # if we've gotten here, it means we never found a frontier cell
        # obviously, something has gone wrong, so complain to the user
        print "find_next_target failed to find a suitable frontier cell"


    def publish_frontiers(self, frontiers):
        # set up frontier_cells in a publishable state
        frontier_cells = GridCells()
        frontier_cells.cell_width = self.mymap.info.resolution
        frontier_cells.cell_height = self.mymap.info.resolution
        frontier_cells.header.frame_id = 'map'
        frontier_cells.header.stamp = rospy.Time.now()

        # process all of the points to allign to the rviz grid, and add them
        for cell in frontiers:
            point = Point()
            cell_coordinates = index_to_point(cell, self.mymap)
            (point.x, point.y) = map_to_world(cell_coordinates, self.mymap)
            frontier_cells.cells.append(point)

        self.frontierCellsPublisher.publish(frontier_cells)



if __name__ == '__main__':
    f = Frontier()
    print "find_frontiers is waiting for a costmap message"
    rospy.wait_for_message('costmap', OccupancyGrid)
    print "find_frontiers is ready"

    rospy.spin()
