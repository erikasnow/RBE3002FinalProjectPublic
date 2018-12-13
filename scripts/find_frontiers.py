#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, GridCells
from map_helper import *
from math import sqrt


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
        closest_frontier = self.find_next_target(frontiers)
        if closest_frontier:
            self.publish_nearest_frontier(closest_frontier)


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

        # if there are no frontier cells left, inform the human operators
        # to save the map and start the next section of the lab
        # (also, don't assume we're done unless we've been running for at least
        #  thirty seconds, because that would be way too optimistic)
        if frontier_list is [] and rospy.get_time() > 30:
            print "\n\n\n\n\n" + "finished mapping! please run:" + "\n\n"
            print "\t\t" + "rosrun map_server map_saver -f [map_file_name]"
            print "\n\n" + "and then start the other launch file"
        print "frontier cells: " + str(frontier_list)
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

        print "looking for the next target in " + str(len(frontier_list)) + " options"

        closest_frontier = None
        closest_distance = 9999999999

        for frontier in frontier_list:

            # calculate the distance from the robot's location (px, py)
            # to the current frontier cell
            frontier_x, frontier_y = map_to_world(index_to_point(frontier, self.mymap), self.mymap)
            delta_x = abs(frontier_x - self.px)
            delta_y = abs(frontier_y - self.py)
            distance = sqrt(delta_x**2 + delta_y**2)

            # if this distance is the smallest we've seen, replace the
            # previous closest cell with this one
            if distance < closest_distance:
                closest_frontier = frontier
                closest_distance = distance

        # if there's nothing in closest_frontier, there must not have been
        # any options in frontier_list, so we've failed
        if closest_frontier is None:
            print "\n\n\n" + "failed to find the next target frontier"

        # otherwise, send the new target frontier
        else:
            print "next target frontier is: " + str(closest_frontier)
            return closest_frontier


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


    def publish_nearest_frontier(self, frontier):
        target = PoseStamped()
        target.header.frame_id = 'map'
        target.header.stamp = rospy.Time.now()
        target.pose.position.x, target.pose.position.y = index_to_point(frontier, self.mymap)
        self.nearestPublisher.publish(target)


if __name__ == '__main__':
    f = Frontier()
    print "find_frontiers is waiting for a costmap message"
    rospy.wait_for_message('costmap', OccupancyGrid)
    print "find_frontiers is ready"

    rospy.spin()
