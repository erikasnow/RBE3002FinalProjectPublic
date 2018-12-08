#!/usr/bin/env python
from math import sqrt, pi, atan2
import roslib
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovariance, Twist, Quaternion, Point, OccupancyGrid
from nav_msgs.msg import Odometry, Path, GridCells
import tf
from tf.transformations import euler_from_quaternion
import std_msgs
from std_msgs import Bool


class Planning:

    def __init__(self):
        """"
        Set up the node here
        """

        rospy.init_node('planning', anonymous=True)

        # subscribes to astar path, scan map, and robot done message
        # is this actually how we need to subscribe to the astar service?
        self.pathSubscriber = rospy.Subscriber('path', Path, self.handle_path)
        self.mapSubscriber = rospy.Subscriber('newmap', OccupancyGrid, self.handle_map)
        # wait a sec... what if we had Robot publish the Pose it just finished moving to? That way we can double-check
        # and only send the next one when the message updates
        self.doneSubscriber = rospy.Subscriber('done', PoseStamped, self.handle_done)

        # publishes waypoint from astar path
        self.wayptPublisher = rospy.Publisher('target', PoseStamped, queue_size=1)

        # global variables (these will be updated in callbacks)
        self.currmap = OccupancyGrid()
        self.currpath = Path()

    def handle_done(self, msg):
        """
        Publishes the next PoseStamped, given the current map and path to goal
        :param msg: PoseStamped
        :return:
        """
        # I'm worried this could potentially write over a new path given by astar

        # check that currpath hasn't updated since robot started moving toward last waypoint
        if self.currpath.poses[0] == msg:
            self.currpath.poses.pop(0)

        # publish the next pose in the path
        self.wayptPublisher.publish(self.currpath.poses[0])

    def handle_map(self, msg):
        """
        Updates the map used for astar
        :param msg: OccupancyGrid
        :return:
        """
        # where do we call astar from here? main?
        self.currmap = msg

    # I think this is probably wrong, but I don't know how to do it for a service
    def handle_path(self, msg):
        """
        Updates the path we get from astar
        :param msg: Path
        :return:
        """
        self.currpath = msg

    def find_nearest_frontier(self):
        """
        Checks current map for nearest frontier and calls astar service to get path to it
        :return:
        """
        # TODO write this mess somehow


if __name__ == '__main__':
    print("make planner")
    p = Planning()
    rospy.spin()
