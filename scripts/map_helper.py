#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from math import *
import tf


def get_neighbors(index, my_map):
    """
        returns the legal neighbors of loc for a 4-connected robot
        :param loc: tuple of location
        :return: list of tuples
    """
    neighbors = []

    map_width = my_map.info.width

    up = index + map_width
    down = index - map_width
    left = index - 1
    right = index + 1

    if is_valid_loc(up, my_map):
        neighbors.append(up)
    if is_valid_loc(down, my_map):
        neighbors.append(down)
    if is_valid_loc(left, my_map):
        neighbors.append(left)
    if is_valid_loc(right, my_map):
        neighbors.append(right)

    return neighbors


def is_valid_loc(index, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    if (my_map.data[index] == 100 or my_map.data[index] == -1):
        return False
    else:
        return True


def convert_location(loc, my_map):
    """converts points to the grid"""
    cell_size = my_map.info.resolution

    x = loc[0]
    y = loc[1]

    x = x - ((x + (cell_size / 2)) % cell_size) + (cell_size / 2)
    y = y - ((y + (cell_size / 2)) % cell_size) + (cell_size / 2)

    newloc = (x, y)

    return newloc


def world_to_map(worldpt, my_map):
    """
        converts a point from the world to the map
        :param worldpt: tuple of point in world
        :return: tuple of converted point
    """
    maporigin = my_map.info.origin.position

    mapx = worldpt[0] - maporigin.x
    mapy = worldpt[1] - maporigin.y
    mappt = (mapx, mapy)

    return mappt


def map_to_world(mappt, my_map):
    """
        converts a point from the map to the world
        :param mappt: tuple of point on mao
        :return: tuple of converted point
    """
    maporigin = my_map.info.origin.position
    worldx = mappt[0] + maporigin.x
    worldy = mappt[1] + maporigin.y
    worldpt = (worldx, worldy)

    return worldpt


def to_poses(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """
    poses = []

    for p in range(len(points)-1):
        thispt = points[p]
        nextpt = points[p+1]

        pt = Pose()
        pt.position.x = thispt[0]
        pt.position.y = thispt[1]

        delx = nextpt[0] - thispt[0]
        dely = nextpt[1] - thispt[1]
        yaw = atan2(dely, delx)
        q = tf.transformations.quaternion_from_euler(yaw, 0, 0, 'rzyx')
        pt.orientation.x = q[0]
        pt.orientation.y = q[1]
        pt.orientation.z = q[2]
        pt.orientation.w = q[3]
        poses.append(pt)

    return poses


def index_to_point(index, my_map):
    """convert a index to a point"""
    width = my_map.info.width
    x = index % width
    y = (index - x) / width
    point = (x, y)
    return point


def point_to_index(location, my_map):
    """convert a point to a index"""

    cell_size = my_map.info.resolution

    # round the values to be evenly divisible by cell_size
    x = location[0] - location[0] % cell_size
    y = location[1] - location[1] % cell_size

    x = int(x/cell_size)
    y = int(y/cell_size)

    width = int(my_map.info.width)

    index = x + (y * width)

    return index


def round_point(point):
    """
    reduces the significant figures of a float to be more manageable
    :param point: Tuple of coordinate
    :param my_map: occupancy grid of the current map
    :return: tuple of converted points
    """
    x = round(point[0], 2)
    y = round(point[1], 2)

    rounded_point = (x,y)
    return rounded_point
