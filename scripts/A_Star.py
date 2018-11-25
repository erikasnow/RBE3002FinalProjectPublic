#!/usr/bin/env python
import rospy
from map_helper import *
from nav_msgs.msg import Path, GridCells
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped, Point
from math import sqrt
from PriorityQueue import *
import std_msgs
#from Astar.srv import *


class A_Star:

    def __init__(self):
        """
            This node runs a service for A* path requests between two
            grid locations. It publishes the frontier and explored cells
            while running the A* algorithm for visualization and debugging.
        """
        rospy.init_node("a_star")

        # occupancy grid for the data
        self.my_map = None
        self.mapSubscriber = rospy.Subscriber('map', OccupancyGrid, self.dynamic_map_client)

        # publish the frontier and explored cells while running A*
        self.frontierPublisher = rospy.Publisher('frontier', GridCells, queue_size=1)
        self.exploredPublisher = rospy.Publisher('explored', GridCells, queue_size=1)

    def handle_astar(self, req):
        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan() with start and goal data
            :return: GetPlan() with a Path()
        """
        start_x = req.start.pose.position.x
        start_y = req.start.pose.position.y
        goal_x = req.goal.pose.position.x
        goal_y = req.goal.pose.position.y

        world_start = (start_x, start_y)
        world_goal = (goal_x, goal_y)
        start = world_to_map(world_start, self.my_map)
        goal = world_to_map(world_goal, self.my_map)

        path_cells = self.a_star(start, goal)
        path = self.create_path(path_cells)
        return GetPlanResponse(path)

    def dynamic_map_client(self, msg):
        """
            Updates my_map with data from the map topic
            :param msg: OccupancyGrid from the map topic
            :return:
        """
        self.my_map = msg

    def a_star(self, start, goal):
        """
            The A* algorithm
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: list of tuples along the optimal path
        """
        print "processing path"
        goal = round_point(goal)
        start = round_point(start)
        print "start point: " + str(start)
        print "end point: " + str(goal)
        print " "
        frontier = PriorityQueue()
        came_from = {}
        cost_so_far = {}

        # add the start cell
        frontier.put(start, 0)
        cost_so_far[start] = 0
        came_from[start] = None

        while not frontier.empty():
            current = frontier.get()
            if (abs(current[0] - goal[0])<0.1 and abs(current[1]-goal[1] < 0.1)):
                came_from[goal] = current
                break

            wavefront = get_neighbors(current, self.my_map)

            for next in wavefront:

                # publish the frontier and explored cells to rviz for debugging
                self.paint_frontier(frontier)
                self.paint_explored(came_from)

                # find the total cost of going from start to next
                cost = cost_so_far[current] + self.move_cost(current, next)

                # if this is the first time exploring next, add it in
                # or, if we just found a faster way to get to next, update it
                if next not in cost_so_far or cost < cost_so_far[next]:
                    cost_so_far[next] = cost
                    priority = cost + self.euclidean_heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current
                    rospy.sleep(0.75)

        # create the path by following came_from from goal back to the start
        path = [goal]
        current = goal
        while current is not start:
            current = came_from[current]
            path.append(current)

        # reverse the path to put it in the right order
        path.reverse()

        return path

    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: Euclidean distance between two points
        """
        x1, y1 = point1
        x2, y2 = point2

        xdelta = abs(x2 - x1)
        ydelta = abs(y2 - y1)

        dist = sqrt((xdelta * xdelta) + (ydelta * ydelta))

        return dist

    def move_cost(self, point1, point2):
        """
            calculate the Manhattan distance between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: Manhattan distance between two points
        """
        x1, y1 = point1
        x2, y2 = point2

        xdelta = abs(x2 - x1)
        ydelta = abs(y2 - y1)

        dist = xdelta + ydelta

        return dist

    def optimize_path(self, path):
        """
            remove redundant points in the path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        pass

    def paint_frontier(self, frontier):
        """
            Paints the A* frontier cells to rviz for debugging
            :param frontier: dictionary storing the frontier cells
            :return:
        """
        frontier_elements = map(lambda foo: foo[1], frontier.elements)

        frontier_cells = GridCells()
        frontier_cells.cell_width = self.my_map.info.resolution
        frontier_cells.cell_height = self.my_map.info.resolution
        frontier_cells.header.frame_id = 'map'
        frontier_cells.header.stamp = rospy.Time.now()

        for cell in frontier_elements:
            cell_as_point = Point()
            cell_as_point.x = map_to_world(cell, self.my_map)[0]
            cell_as_point.y = map_to_world(cell, self.my_map)[1]
            frontier_cells.cells.append(cell_as_point)

        self.frontierPublisher.publish(frontier_cells)

    def paint_explored(self, came_from):
        """
            finds the explored cells from A*'s "came_from"
            dictionary and paints it to rviz for debugging
            :param came_from: came_from dictionary used by A*
            :return:
        """
        explored_cells = GridCells()
        explored_cells.cell_width = self.my_map.info.resolution
        explored_cells.cell_height = self.my_map.info.resolution
        explored_cells.header.frame_id = 'map'
        explored_cells.header.stamp = rospy.Time.now()

        for cell in came_from:
            cell_as_point = Point()
            cell_as_point.x = map_to_world(cell, self.my_map)[0]
            cell_as_point.y = map_to_world(cell, self.my_map)[1]
            explored_cells.cells.append(cell_as_point)

        self.exploredPublisher.publish(explored_cells)

    def create_path(self, points):
        """
            Publishes a Path() containing the waypoints along the path
            :param points: list of tuples of the path
            :return:
        """
        path = Path()
        path.header.frame_id = 'map'

        # copy all the points as PoseStamped() into the path
        for point in points:

            # get the real-world coordinates of the point relative to the
            # origin of the map, from the tuple of grid coordinates A* returns
            world_x, world_y = map_to_world(point, self.my_map)

            # make a PoseStamped at the real-world coordinates of the point
            # currently has the orientation set to the default, to be ignored
            pose = PoseStamped()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y

            # append the new pose to the list of poses that make up the path
            path.poses.append(pose)

        return path


if __name__ == '__main__':
    alg = A_Star()
    s = rospy.Service('astar', GetPlan, alg.handle_astar)
    print "A* service is running"
    rospy.spin()
