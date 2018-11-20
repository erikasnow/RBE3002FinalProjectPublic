#!/usr/bin/env python
import rospy
import sys
from map_helper import *
#from astar.srv import *

def process_pose_message(msg, x_position, y_position):
    """
        processes a start or end
        :param msg: the message from rviz with a start or goal
        :param x_position: the x position from the message
        :param y_position: the y position from the message
    """
    header = msg.header
    header.stamp = rospy.Time.now()

    point = Point()
    point.x = x_position
    point.y = y_position

    world_loc = (point.x, point.y)
    grid_loc = convert_location(world_loc, my_map)
    point.x, point.y = grid_loc

    points = []
    points.append(point)

    gridcells = GridCells()
    gridcells.header = header
    gridcells.cell_width = my_map.info.resolution
    gridcells.cell_height = my_map.info.resolution
    gridcells.cells = points

    return grid_loc, gridcells

def handle_start_pose(msg):
    x_position = msg.pose.pose.position.x
    y_position = msg.pose.pose.position.y

    global start_loc
    start_loc, initcell = process_pose_message(msg, x_position, y_position)
    initPublisher.publish(initcell)

def handle_goal(msg):
    x_position = msg.pose.position.x
    y_position = msg.pose.position.y

    goal_loc, goalcell = process_pose_message(msg, x_position, y_position)
    goalPublisher.publish(goalcell)
    # todo: call A* here with start_loc and goal_loc

def handle_map_updates(msg):
    global my_map
    my_map = msg


if __name__ == '__main__':

    rospy.init_node("main")

    start_loc = (0, 0)

    # occupancy grid for the map data
    my_map = None
    mapSubscriber = rospy.Subscriber('map', OccupancyGrid, handle_map_updates)
    # subscribe to rviz start and goal cells
    start_pose_subscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, handle_start_pose)
    goal_subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, handle_goal)
    # publish start and goal cells
    initPublisher = rospy.Publisher('initcell', GridCells, queue_size=1)
    goalPublisher = rospy.Publisher('goalcell', GridCells, queue_size=1)

    rospy.spin()
