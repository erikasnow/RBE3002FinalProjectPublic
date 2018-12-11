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
from map_helper import *


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
        self.odomSubscriber = rospy.Subscriber('odom', Odometry, self.handle_odom)

        # publishes waypoint from astar path
        self.wayptPublisher = rospy.Publisher('target', PoseStamped, queue_size=1)

        # global variables (these will be updated in callbacks)
        self.currmap = OccupancyGrid()
        self.currpath = Path()
        self.px = 0
        self.py = 0

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

    def handle_odom(self, msg):
        """
        Updates the current location of the robot
        :param msg: Odom
        :return:
        """
        self.px = msg.pose.pose.position.x
        self.py = msg.pose.pose.position.y

    def find_nearest_frontier(self):
        """
        Checks current map for nearest frontier and returns centroid of that frontier
        :return: location of nearest frontier cell
        """
        # TODO test somehow
        map = self.currmap
        nearest_index = -1
        nearest_distance = -1
        robotx = self.px
        roboty = self.py

        robotx, roboty = convert_location({robotx, roboty}, map)

        # should this return a point or an index?
        for cell in map.data:
            index = map.data.index(cell)
            no_wall = True
            width = int(map.info.width)

            up = index + width  # I think the grid starts from bottom left
            down = index - width  # I think the grid starts from bottom left
            left = index - 1
            right = index + 1

            # don't go to any location next to a wall
            if map.data[up] > 50:
                no_wall = False
            if map.data[down] > 50:
                no_wall = False
            if map.data[left] > 50:
                no_wall = False
            if map.data[right] > 50:
                no_wall = False

            if cell == -1 and no_wall:
                ix, iy = index_to_point(map.data.index(cell), map)
                deltax = abs(ix - robotx)
                deltay = abs(iy - roboty)
                dist = sqrt((deltax * deltax) + (deltay * deltay))
                if nearest_distance == -1 or dist < nearest_distance:
                    # replace values
                    nearest_index = map.data.index(cell)
                    nearest_distance = dist

        # if there is no frontier, print that we've got the full map, and navigate back to the start location
        if nearest_index == -1:
            print("map is complete")
        return nearest_index


if __name__ == '__main__':
    print("make planner")
    p = Planning()
    rospy.spin()
