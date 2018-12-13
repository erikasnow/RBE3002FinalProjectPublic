#!/usr/bin/env python
import rospy
import sys
from map_helper import *
from nav_msgs.srv import GetPlan, GetPlanResponse
from nav_msgs.msg import Odometry
from Robot import *
from math import pi


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
    global goal_pose
    goal_pose = msg
    x_position = msg.pose.position.x
    y_position = msg.pose.position.y

    #goalcell = process_pose_message(msg, x_position, y_position)
    goal_loc, goalcell = process_pose_message(msg, x_position, y_position)
    goalPublisher.publish(goalcell)


def call_astar():
    global goal_pose
    global start_pose

    try:
        print "Trying A* service"
        get_plan = GetPlan()
        get_plan.start = start_pose
        get_plan.goal = goal_pose
        astar_service = rospy.ServiceProxy('astar', GetPlan)
        response = astar_service(start_pose, goal_pose, 0.1)
        global path
        path = response.plan
        pathPublisher.publish(path)
        #targetPublisher.publish(path.poses[1].pose)
    except rospy.ServiceException, e:
        print "\nA* service call failed:\n" + str(e)


def handle_map_updates(msg):
    global my_map
    my_map = msg


# grabs pose of the turtlebot
def odom_callback(msg):
    """
    update the state of the robot
    :type msg: Odom
    :return:
    """
    global start_pose
    if(start_pose.pose.position.x is not msg.pose.pose.position.x or
            start_pose.pose.position.y is not msg.pose.pose.position.y):

        robot_loc = Point()
        robot_loc.x = start_pose.pose.position.x
        robot_loc.y = start_pose.pose.position.y

        world_loc = (robot_loc.x, robot_loc.y)
        grid_loc = convert_location(world_loc, my_map)
        robot_loc.x, robot_loc.y = grid_loc

        gridcell_points = []
        gridcell_points.append(robot_loc)

        gridcells = GridCells()
        gridcells.header = msg.header
        gridcells.header.stamp = rospy.Time.now()
        gridcells.cell_width = my_map.info.resolution
        gridcells.cell_height = my_map.info.resolution
        gridcells.cells = gridcell_points

        locationPublisher.publish(gridcells)

    start_pose.pose.position.x = msg.pose.pose.position.x
    start_pose.pose.position.y = msg.pose.pose.position.y


def handle_robot_done(msg):
    global last_pose_finished
    last_pose_finished = msg


def check_new_frontier(curr_frontier):
    global goal_pose
    if goal_pose == curr_frontier:
        return False
    else:
        return True


if __name__ == '__main__':

    rospy.init_node("main")
    print "main node initialized"
    rospy.sleep(1)

    # variable to store the robot's location, initialized from launch file
    start_pose = PoseStamped()
    start_pose.pose.position.x = rospy.get_param('~x_pos', 0.0)
    start_pose.pose.position.y = rospy.get_param('~y_pos', 0.0)

    # occupancy grid for the map data
    # make sure the map is being published before continuing, since
    # odomSubscriber relies on it having data before it is run
    my_map = None
    mapSubscriber = rospy.Subscriber('costmap', OccupancyGrid, handle_map_updates)
    rospy.wait_for_message('costmap', OccupancyGrid)

    # publish start, goal, and current location cells to rviz
    initPublisher = rospy.Publisher('initcell', GridCells, queue_size=1)
    goalPublisher = rospy.Publisher('goalcell', GridCells, queue_size=1)
    locationPublisher = rospy.Publisher('location', GridCells, queue_size=1)

    # subscribe to odom updates to update the start cell
    odomSubscriber = rospy.Subscriber('odom', Odometry, odom_callback)

    # publish the optimal path
    pathPublisher = rospy.Publisher('path', Path, queue_size=1)

    # publish the navigation target
    targetPublisher = rospy.Publisher('target', Pose, queue_size=1)

    # subscribe to rviz start and goal cells
    start_pose_subscriber = rospy.Subscriber('initialpose', PoseWithCovarianceStamped, handle_start_pose)
    fgoal_subscriber = rospy.Subscriber('nearest_frontier', PoseStamped, handle_goal)
    goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, handle_goal)
    # subscribe to robot done messages
    doneSubscriber = rospy.Subscriber('done', Pose, handle_robot_done)

    path = Path()
    currpose = Pose()
    last_pose_finished = Pose()
    goal_pose = PoseStamped()

    print "waiting for A* service"
    rospy.wait_for_service('astar')
    print "main sees A* service"

    # important variables
    x = start_pose.pose.position.x
    y = start_pose.pose.position.y

    rospy.wait_for_message('nearest_frontier', PoseStamped)
    print("got nearest_frontier")

    currgoal = goal_pose

    call_astar()

    currpath = path
    currpathcopy = path
    if len(path.poses) != 0:
        currpose = currpath.poses[1].pose

    targetPublisher.publish(currpathcopy.poses[1].pose)
    print("")
    print("PUBLISHED FIRST TARGET")

    # main loop
    while not rospy.is_shutdown():
        # if path hasn't changed and robot finished last pose it moved to
        if currpath.poses == path.poses and currpose == last_pose_finished:
            print("PATH IS SAME, GET NEXT POSE")
            currpathcopy.poses.pop(0)

            # if there are still poses in the path, keep publishing
            if currpathcopy.poses != []:

                # publish next waypoint for robot
                targetPublisher.publish(currpathcopy.poses[0].pose)
                print("publish target")

                # wait for response
                while last_pose_finished == currpose:
                    rospy.wait_for_message('done', Pose)
                currpose = last_pose_finished
                print("robot is done moving")
                if check_new_frontier(currgoal):
                    # if it's updated, then call astar and grab a new path
                    call_astar()
                    print("should now have path")
                    currgoal = goal_pose

                    currpath = path
                    currpathcopy = path

                    #while last_pose_finished == currpose:  # astar publishes first target location
                    #    rospy.wait_for_message('done', Pose)
                    #currpose = last_pose_finished

            # if no more poses, break out of while loop and save the map
            else:
                print("path is finished -- wait for new path")
                while currpath.poses == path.poses:
                    rospy.sleep(.5)  # hopefully only pauses this node for half a second

        elif currpath.poses != path.poses:
            print("currpath.poses != path.poses")
            # update the current path
            currpath = path
            currpathcopy = path
            currpose = last_pose_finished

        else:
            print("PATH IS SAME, TARGET IS NOT")
            currpose = last_pose_finished


    rospy.spin()

