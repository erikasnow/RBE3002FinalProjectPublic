#!/usr/bin/env python
import rospy
from map_helper import *

class A_Star:

    def __init__(self):

        """
            This node handle A star paths requests.
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
        self.pathPublisher = rospy.Publisher('path', GridCells, queue_size=1)

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
            A*
            This is where the A* algorithum belongs
            :param start: tuple of start pose
            :param goal: tuple of goal pose
            :return: dict of tuples
        """
        pass

      
    def euclidean_heuristic(self, point1, point2):
        """
            calculate the dist between two points
            :param point1: tuple of location
            :param point2: tuple of location
            :return: dist between two points
        """
    pass

    def move_cost(self, current, next):
        """
              calculate the dist between two points
              :param current: tuple of location
              :param next: tuple of location
              :return: dist between two points
        """
        pass


    def reconstruct_path(self, start, goal, came_from):
        """
            Rebuild the path from a dictionary
            :param start: starting key
            :param goal: starting value
            :param came_from: dictionary of tuples
            :return: list of tuples
       """
        pass
  

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
            Create a Path() and publishes the cells if Paint is true
            :param points: list of tuples of the path
            :return: Path()
        """
        self.pathPublisher.publish(points)  # pass in dummy data for now

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


if __name__ == '__main__':
    alg = A_Star()
    print("made algorithm")
    rospy.sleep(1)
    rospy.spin()
    pass
