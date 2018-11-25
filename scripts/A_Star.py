#!/usr/bin/env python
import rospy
from map_helper import *
from nav_msgs.msg import Path, GridCells
from geometry_msgs.msg import PoseStamped, Point
from math import sqrt
from PriorityQueue import *
import std_msgs


class A_Star:

    def __init__(self):
        """
            This node handles A* path requests
            It is accessed using a service call. It can the publish grid cells
            to show the frontier, closed and path.
        """

        rospy.init_node("a_star")  # start node
        # for setting initial position on rviz
        self.initSubscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.display_start)
        self.initPublisher = rospy.Publisher('initcell', GridCells, queue_size=1)

        # for setting goal position on rviz
        self.goalSubscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.display_goal)
        self.goalPublisher = rospy.Publisher('goalcell', GridCells, queue_size=1)

        # make an occupancy grid for the data
        self.my_map = None
        self.mapSubscriber = rospy.Subscriber('map', OccupancyGrid, self.dynamic_map_client)  # is this right?

        # for setting the frontier on rviz
        self.frontierPublisher = rospy.Publisher('frontier', GridCells, queue_size=1)

        # for showing the explored cells during A*
        self.exploredPublisher = rospy.Publisher('explored', GridCells, queue_size=1)

        # for setting the path on rviz
        self.pathPublisher = rospy.Publisher('path', Path, queue_size=1)
        #self.pathPublisher = rospy.Publisher('path', GridCells, queue_size=1)

        start = (0,0) # variable to store the start cell todo: remove after testing

    def handle_a_star(self, req):

        """
            service call that uses A* to create a path.
            This can be altered to also grab the orentation for better heuristic
            :param req: GetPlan
            :return: Path()
        """
        pass

    def dynamic_map_client(self, msg):

        """
            Service call to get map and set class variables
            This can be changed to call the expanded map
            :return:
        """
        print("grab occupancy grid")
        self.my_map = msg  # I think I just made a variable with the current map from the map server?

    def a_star(self, start, goal):
        """
            The A* algorithm
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: list of tuples along the optimal path
        """

        frontier = PriorityQueue()
        came_from = {}
        cost_so_far = {}

        # add the start cell
        frontier.put(start, 0)
        cost_so_far[start] = 0
        came_from[start] = None

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            wavefront = get_neighbors(current, self.my_map)
            #self.paint_cells(wavefront, current)

            #print "get_neighbors returns:\n" + str(get_neighbors(current, self.my_map))
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

        # create the path by following came_from from goal back to the start
        path = [goal]
        current = goal
        #print "came_from is:\n" + str(came_from)
        while current is not start:
            current = came_from[current]
            path.append(current)

        path.reverse() # reverse the path to put it in the right order

        return path


    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
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
            :param current: tuple of location
            :param next: tuple of location
            :return: Manhattan distance between two points
        """
        x1, y1 = point1
        x2, y2 = point2

        xdelta = abs(x2 - x1)
        ydelta = abs(y2 - y1)

        dist = xdelta + ydelta
        return dist


#    def reconstruct_path(self, start, goal, came_from):
#        """
#            Rebuild the path from a dictionary
#            :param start: starting key
#            :param goal: starting value
#            :param came_from: dictionary of tuples
#            :return: list of tuples
#        """
#        pass

    def optimize_path(self, path):
        """
            remove redundant points in the path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        print"entered optimize"

        print("old path: ")
        print(path.poses)

        index = 0
        currx = path.poses[0].pose.position.x
        curry = path.poses[0].pose.position.y

        newpath = Path()
        newpath.header.frame_id = 'map'
        newpath.poses.append(path.poses[0])  # add first node to cleaned path

        path.poses.pop(0)  # get rid of first node

        for pose in path.poses:
            if(currx != pose.pose.position.x):
                if(curry != pose.pose.position.y):
                    newpath.poses.append(path.poses[index-1])  # add the previous node
                    currx = pose.pose.position.x  # x needs to change, but y should stay the same, I think
                    index -= 1

            elif(curry != pose.pose.position.y):
                if(currx != pose.pose.position.x):
                    newpath.poses.append(path.poses[index-1])  # add the previous node
                    curry = pose.pose.position.y  # y needs to change, but x should stay the same, I think
                    index -= 1

            index += 1

        print("new path: ")
        print(newpath.poses)
        return newpath

    def paint_frontier(self, frontier):
        """
            finds the frontier cells from A*'s "came_from"
            dictionary and paints it to rviz for debugging
            :param came_from: came_from dictionary used by A*
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

    def paint_cells(self, frontier, came_from):
        # type: (list, tuple) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        for f_cell in frontier:
            point = Point()
            points = []
            loc = convert_location(f_cell, self.my_map)
            point.x, point.y = loc
            points.append(point)

            header = std_msgs.msg.Header()
            header.frame_id = str(0)

            frontier_cell = GridCells()
            frontier_cell.cells = points
            frontier_cell.cell_width = self.my_map.info.resolution
            frontier_cell.cell_height = self.my_map.info.resolution
            header.stamp = rospy.Time.now()
            frontier_cell.header = header

            self.wavefrontPublisher.publish(frontier_cell)

        point = Point()
        points = []
        loc = convert_location(came_from, self.my_map)
        point.x, point.y = loc
        points.append(point)

        header = std_msgs.msg.Header()
        header.frame_id = str(0)

        searched_cell = GridCells()
        searched_cell.cells = points
        searched_cell.cell_width = self.my_map.info.resolution
        searched_cell.cell_height = self.my_map.info.resolution
        header.stamp = rospy.Time.now()
        searched_cell.header = header

        self.searchedPublisher.publish(searched_cell)

    def publish_path(self, points):
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

            # make a PoseStamped at the real-world coorinates of the point
            # currently has the orientation set to the default, to be ignored
            pose = PoseStamped()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y

            # append the new pose to the list of poses that make up the path
            path.poses.append(pose)

        # newpath = self.optimize_path(path)  # currently broken

        # self.pathPublisher.publish(newpath)
        self.pathPublisher.publish(path)

#        path_cells = []
#        grid = GridCells()
#
#        # copy all the poses in the path into Points for the GridCells message
#        for pose in path.poses:
#            cell_loc = pose.pose.position
#            path_cells.append(cell_loc)
#
#        grid.cells = path_cells
#        self.pathPublisher.publish(grid)

    def display_start(self, msg):
        """
        Get pose of initial location from rviz click and paint corresponding cell blue
        :param msg:
        :return:
        """
        header = msg.header
        print("msg.header.seq = " + str(msg.header.seq))
        header.stamp = rospy.Time.now()

        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y

        loc = (point.x, point.y)
        print("old start point:\n" + str(point))

        newloc = convert_location(loc, self.my_map)

        point.x, point.y = newloc
        print("new start point:\n" + str(point))

        points = []
        points.append(point)

        initcell = GridCells()
        initcell.header = header
        initcell.cell_width = self.my_map.info.resolution
        initcell.cell_height = self.my_map.info.resolution
        initcell.cells = points

        self.start = newloc # todo: remove this after testing
        self.initPublisher.publish(initcell)

    def display_goal(self, msg):
        """
        Get pose of goal location from rviz click and paint corresponding cell red
        :param msg:
        :return:
        """
        header = msg.header
        print("msg.header.seq = " + str(msg.header.seq))
        header.stamp = rospy.Time.now()

        point = Point()
        point.x = msg.pose.position.x
        point.y = msg.pose.position.y

        loc = (point.x, point.y)
        print("old goal point:\n" + str(point))

        newloc = convert_location(loc, self.my_map)

        point.x, point.y = newloc
        print("new goal point:\n" + str(point))

        points = []
        points.append(point)

        goalcell = GridCells()
        goalcell.header = header
        goalcell.cell_width = self.my_map.info.resolution
        goalcell.cell_height = self.my_map.info.resolution
        goalcell.cells = points

        self.goalPublisher.publish(goalcell)

        # todo: remove this section after testing:
        goal = newloc
        print "processing path"
        path = self.a_star(world_to_map(self.start, self.my_map), world_to_map(goal, self.my_map))
        print "path is:\n" + str(path)
        self.publish_path(path)


if __name__ == '__main__':
    alg = A_Star()
    print("made algorithm")
    rospy.sleep(1)

    rospy.spin()
    pass
