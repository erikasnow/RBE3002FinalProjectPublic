#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from math import *
import tf


# my_map is an occupancy grid


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc for a 4-connected robot
        :param loc: tuple of location
        :return: list of tuples
    """
    neighbors = []

    cell_step = my_map.info.resolution

    up = (loc[0], loc[1] + cell_step)
    down = (loc[0], loc[1] - cell_step)
    left = (loc[0] - cell_step, loc[1])
    right = (loc[0] + cell_step, loc[1])

    if is_valid_loc(up, my_map):
        neighbors.append(up)

    if is_valid_loc(down, my_map):
        neigbors.append(down)

    if is_valid_loc(left, my_map):
        neighbors.append(left)

    if is_valid_loc(right, my_map):
        neighbors.append(right)

    return neighbors


def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    x = loc[0]
    y = loc[1]

    width = my_map.info.width
    origin = my_map.info.origin
    index = point_to_index(loc, my_map)

    xupbnd = width - origin.x
    xlobnd = -1 * (width - xupbnd)
    yupbnd = width - origin.y
    ylobnd = -1 * (width - yupbnd)

    if (x < xlobnd or x > xupbnd):
        return False
    elif (y < ylobnd or y > yupbnd):
        return False
    elif (my_map.data[index] == 100 or my_map.data[index] == -1):
        return False
    else:
        return True


def convert_location(loc, my_map):
    """converts points to the grid"""
    cell_size = my_map.info.resolution
    loc2 = world_to_map(loc, my_map)
    x = loc2[0]
    y = loc2[1]

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
    maporigin = my_map.info.origin
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
    maporigin = my_map.info.origin
    worldx = mappt[0] + maporigin.x
    worldy = mappt[1] + maporigin.y
    worldpt = (worldx, worldy)
    return worldpt


def to_cells(points, my_map):
    """
        Creates a GridCell() for Rviz distplay
        :param points: list of tuples
        :return: GridCell()
    """
    



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
    x = int(location[0])
    y = int(location[1])
    width = int(my_map.info.width)
    index = x + (y * width)
    return index
