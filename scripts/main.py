#!/usr/bin/env python
import rospy
import sys
from map_helper import *
from nav_msgs.srv import GetPlan, GetPlanResponse


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
    """
        Get pose of start from rviz, and paint that cell
        :param msg:
        :return:
    """
    x_position = msg.pose.pose.position.x
    y_position = msg.pose.pose.position.y

    start_loc, initcell = process_pose_message(msg, x_position, y_position)
    initPublisher.publish(initcell)

    # copy the message data into start_pose
    global start_pose
    start_pose.header = msg.header
    start_pose.pose.position.x = start_loc[0]
    start_pose.pose.position.y = start_loc[1]


def handle_goal(msg):
    """
        Get pose of goal from rviz, paint that cell, and run A*
        :param msg:
        :return:
    """
    goal_pose = msg
    x_position = msg.pose.position.x
    y_position = msg.pose.position.y

    #goalcell = process_pose_message(msg, x_position, y_position)
    goal_loc, goalcell = process_pose_message(msg, x_position, y_position)
    goalPublisher.publish(goalcell)

    try:
        get_plan = GetPlan()
        get_plan.start = start_pose
        get_plan.goal = goal_pose
        astar_service = rospy.ServiceProxy('astar', GetPlan)
        response = astar_service(start_pose, goal_pose, 0.1)
        path = response.plan
        pathPublisher.publish(path)
    except rospy.ServiceException, e:
        print "\nA* service call failed:\n" + str(e)


def handle_map_updates(msg):
    global my_map
    my_map = msg


if __name__ == '__main__':

    rospy.init_node("main")
    print "main node initialized"


    start_pose = PoseStamped()
    start_pose.pose.position.x = rospy.get_param('~x_pos', 0.0) # get the initialization location from the launch file
    start_pose.pose.position.y = rospy.get_param('~y_pos', 0.0)

    # occupancy grid for the map data
    my_map = None
    mapSubscriber = rospy.Subscriber('map', OccupancyGrid, handle_map_updates)

    # subscribe to rviz start and goal cells
    start_pose_subscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, handle_start_pose)
    goal_subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, handle_goal)

    # publish start and goal cells
    initPublisher = rospy.Publisher('initcell', GridCells, queue_size=1)
    goalPublisher = rospy.Publisher('goalcell', GridCells, queue_size=1)

    # publish the optimal path
    pathPublisher = rospy.Publisher('path', Path, queue_size=1)

    print "waiting for A* service"
    rospy.wait_for_service('astar')
    print "main sees A* service"

    rospy.spin()
