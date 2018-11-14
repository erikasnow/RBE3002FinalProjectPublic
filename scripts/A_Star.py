#!/usr/bin/env python
import rospy
from map_helper import *
from nav_msgs import Path, PoseStamped
from math import sqrt
import PriorityQueue


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

        # for setting the wavefront on rviz
        self.wavefrontPublisher = rospy.Publisher('frontier', GridCells, queue_size=10)  # I don't actually know what this should be

        # for setting the path on rviz
        self.pathPublisher = rospy.Publisher('path', Path, queue_size=1)

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

        while not frontier.empty():
            current = frontier.get()

            if current == goal:
                break

            for next in get_neighbors(current):

                # find the total cost of going from start to next
                cost = cost_so_far[current] + move_cost(current, next)

                # if this is the first time exploring next, add it in
                # or, if we just found a faster way to get to next, update it
                if next not in cost_so_far or cost < cost_so_far[next]:
                    cost_so_far[next] = cost
                    priority = cost + euclidean_heuristic(next, goal)
                    frontier.put(next, priority)
                    came_from[next] = current

        # create the path by following came_from from goal back to the start
        path = [goal]
        current = goal
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
            remove redundant points in hte path
            :param path: list of tuples
            :return: reduced list of tuples
        """
        pass

    def paint_cells(self, frontier, came_from):
        # type: (list, list) -> None
        """
            published cell of A* to Rviz
            :param frontier: tuples of the point on the frontier set
            :param came_from: tuples of the point on the closed set
            :return:
        """
        pass

    def publish_path(self, points):
        """
            Publishes a Path() containing the waypoints along the path
            :param points: list of tuples of the path
            :return:
        """
        path = Path()

        # copy all the points as PoseStamped() into the path
        for point in points:

            # get the real-world coordinates of the point relative to the
            # origin of the map, from the tuple of grid coordinates A* returns
            world_x, world_y = map_to_world(point[0], point[1], self.my_map)

            # make a PoseStamped at the real-world coorinates of the point
            # currenlty has the orientation set to the default, to be ignored
            pose = PoseStamped()
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y

            # append the new pose to the list of poses that make up the path
            path.poses.append(pose)

        self.pathPublisher.publish(path)

    def display_start(self, msg):
        """
        Get pose of initial location from rviz click and paint corresponding cell blue
        :param msg:
        :return:
        """
        header = msg.header
        header.stamp = rospy.Time.now()

        point = Point()
        point.x = msg.pose.pose.position.x
        point.y = msg.pose.pose.position.y

        loc = (point.x, point.y)
        print("old point: " + str(point))

        newloc = convert_location(loc, self.my_map)

        point.x, point.y = newloc
        print("new point: " + str(point))

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
        header.stamp = rospy.Time.now()

        point = Point()
        point.x = msg.pose.position.x
        point.y = msg.pose.position.y

        loc = (point.x, point.y)
        print("old point: " + str(point))

        newloc = convert_location(loc, self.my_map)

        point.x, point.y = newloc
        print("new point: " + str(point))

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
        self.a_star(start, goal)


if __name__ == '__main__':
    alg = A_Star()
    print("made algorithm")
    rospy.sleep(1)
    #point1 = (0, 0)
    #point2 = (-3, 4)
    #print(alg.euclidean_heuristic(point1, point2))
    rospy.spin()
    pass
