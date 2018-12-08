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


class Planning:

    def __init__(self):
        """"
        Set up the node here
        """
        rospy.init_node('Planning', anonymous=True)
        # subscribes to astar path, scan map, and robot done message
        # is this actually how we need to subscribe to the astar service?
        self.pathSubscriber = rospy.Subscriber('path', Path, self.handle_path)  # TODO write callback
        self.mapSubscriber = rospy.Subscriber('newmap', OccupancyGrid, self.handle_map)  # TODO write callback
        # TODO make done subscriber

        # publishes waypoint from astar path
        self.wayptPublisher = rospy.Publisher('target', PoseStamped, queue_size=1)


if __name__ == '__main__':
        rospy.spin()