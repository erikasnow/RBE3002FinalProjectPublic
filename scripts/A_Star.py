#!/usr/bin/env python
import rospy
from map_helper import *
from nav_msgs.msg import Path, GridCells
from nav_msgs.srv import GetPlan, GetPlanResponse
from geometry_msgs.msg import PoseStamped, Point
from math import sqrt
from PriorityQueue import *
import std_msgs


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
        self.mapSubscriber = rospy.Subscriber('costmap', OccupancyGrid, self.dynamic_map_client)

        # publish the frontier and explored cells while running A*
        self.frontierPublisher = rospy.Publisher('frontier', GridCells, queue_size=1)
        self.exploredPublisher = rospy.Publisher('explored', GridCells, queue_size=1)

    def handle_astar(self, req):
        """
            service call that uses A* to create a path
            :param req: GetPlan() with start and goal data
            :return: GetPlan() with a Path()
        """
        start_x = req.start.pose.position.x
        start_y = req.start.pose.position.y
        goal_x = req.goal.pose.position.x
        goal_y = req.goal.pose.position.y

        world_start = (start_x, start_y)
        goal = (goal_x, goal_y)
        start = world_to_map(world_start, self.my_map)
        #goal = world_to_map(world_goal, self.my_map)

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
        print "start: " + str(start)
        print "goal: " + str(goal)
        goal = point_to_index(round_point(goal),self.my_map)
        start = point_to_index(round_point(start), self.my_map)
        print "start index: " + str(start)
        print "goal index: " + str(goal)

        frontier = PriorityQueue()
        came_from = {}
        cost_so_far = {}

        print "initialized structs"

        # add the start cell
        frontier.put(start, 0)
        cost_so_far[start] = 0
        came_from[start] = None

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            wavefront = filter_valid_arr(get_neighbors(current, self.my_map, True), self.my_map)

            for next in wavefront:
                # publish the frontier and explored cells to rviz for debugging
                self.paint_frontier(frontier)
                self.paint_explored(came_from)
                rospy.sleep(0.01)

                # find the total cost of going from start to next
                cost = cost_so_far[current] + self.move_cost(current, next)
                # if this is the first time exploring next, add it in
                # or, if we just found a faster way to get to next, update it
                if next not in cost_so_far or cost < cost_so_far[next]:
                    cost_so_far[next] = cost
                    priority = cost + self.euclidean_heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        # create the path by following came_from from goal back to the start
        path = [goal]
        current = goal
        while current is not start:
            current = came_from[current]
            path.append(current)

        # reverse the path to put it in the right order
        path.reverse()
        print "path found"
        print str(path)
        return path

    def euclidean_heuristic(self, index1, index2):
        """
            calculate the dist between two points
            :param index1: index of cell in occupancy grid
            :param index2: index of cell in occupancy grid
            :return: Euclidean distance between two points
        """
        x1,y1 = index_to_point(index1,self.my_map)
        x2,y2 = index_to_point(index2,self.my_map)

        xdist = abs(x2 - x1)
        ydist = abs(y2 - y1)

        dist = sqrt(pow(xdist,2) + pow(ydist,2))
        return dist

    def move_cost(self, index1, index2):
        """
            calculate the Manhattan distance between two points
            :param index1: tuple of location
            :param index2: tuple of location
            :return: Manhattan distance between two points
        """
        x1,y1 = index_to_point(index1,self.my_map)
        x2,y2 = index_to_point(index2,self.my_map)

        y = abs(y1 - y2)
        x = abs(x1 - x2)
        cost = sqrt(pow(x,2) + pow(y,2))
        # cost1 = self.my_map.data[index1]
        # cost2 = self.my_map.data[index2]
        # cost = cost2 - cost1
        #print "The cost to travel between " + str(index1) + " and " + str(index2) + " is " + str(cost)
        return cost

    def optimize_path(self, path):
        """
            remove redundant points in the path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        newpath = Path()
        newpath.header.frame_id = 'map'

        pose_list = path.poses
        prev_del_x = 0
        prev_del_y = 0

        for index in range(len(pose_list)):
            if index == pose_list.index(pose_list[-1]):
                newpath.poses.append(pose_list[-1])
                break
            else:
                del_x = pose_list[index].pose.position.x - pose_list[index + 1].pose.position.x
                del_y = pose_list[index].pose.position.y - pose_list[index + 1].pose.position.y

                if del_x != prev_del_x or del_y != prev_del_y:
                    newpath.poses.append(pose_list[index])

                prev_del_x = del_x
                prev_del_y = del_y
        return newpath

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
            cell = index_to_point(cell, self.my_map)
            world_pt = map_to_world(cell, self.my_map)
            cell_as_point.x = world_pt[0]
            cell_as_point.y = world_pt[1]
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
            cell = index_to_point(cell, self.my_map)
            world_pt = map_to_world(cell, self.my_map)
            cell_as_point.x = world_pt[0]
            cell_as_point.y = world_pt[1]
            explored_cells.cells.append(cell_as_point)

        self.exploredPublisher.publish(explored_cells)

    def create_path(self, points):
        """
            Converts the path found by A* from an array of tuples
            to a Path() containing the waypoints along the path
            :param points: list of tuples of the path
            :return: optimized Path()
        """
        path = Path()
        path.header.frame_id = 'map'

        # copy all the points as PoseStamped() into the path
        for index in points:

            # get the real-world coordinates of the point relative to the
            # origin of the map, from the tuple of grid coordinates A* returns
            point = index_to_point(index,self.my_map)
            world_pt = map_to_world(point, self.my_map)
            #world_pt = round_point((world_x,world_y))
            # make a PoseStamped at the real-world coordinates of the point
            # currently has the orientation set to the default, to be ignored
            pose = PoseStamped()
            pose.pose.position.x = world_pt[0]
            pose.pose.position.y = world_pt[1]

            # append the new pose to the list of poses that make up the path
            path.poses.append(pose)
        newpath = self.optimize_path(path)
        return newpath
        #return path

if __name__ == '__main__':
    alg = A_Star()
    s = rospy.Service('astar', GetPlan, alg.handle_astar)
    print "A* service is running"
    rospy.spin()
