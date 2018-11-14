#!/usr/bin/env python
import sys
import rospy
from nav_msgs.msg import OccupancyGrid, GridCells, Path
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose

# my_map is an occupancy grid


def get_neighbors(loc, my_map):
    """
        returns the legal neighbors of loc
        :param loc: tuple of location
        :return: list of tuples
    """
    if (is_valid_loc(loc, my_map)):

  

def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """
    newloc = convert_location(loc, my_map)
    maporigin = my_map.info.origin.position

    x = newloc[0]
    y = newloc[1]



    xupbnd = my_map.info.width - maporigin.x
    xlobnd = my_map.info.width - xupbnd
    yupbnd = my_map.info.height - maporigin.y
    ylobnd = my_map.info.height - yupbnd

    index = point_to_index(newloc, my_map)
    free_thres = 50         # arbitrary value, not sure how to implement with given occupancy grid
    # first test to see if it's within the map
    if(x >  xupbnd or  x < xlobnd):
        return False
    elif(y > yupbnd or y < ylobnd):
        return False
    # test to determine if the point is within an obstacle or unreachable
    elif(my_map.data[index] > free_thresh):
        return False
    elif(my_map.data == -1): #is Unreachable Case
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

def world_to_map(x, y, my_map):
    """
        converts a point from the world to the map
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """

def map_to_world(x, y, my_map):
    """
        converts a point from the map to the world
        :param x: float of x position
        :param y: float of y position
        :return: tuple of converted point
    """


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


def index_to_point(point, my_map):
    """convert a point to a index"""
    width = my_map.info.width
    

def point_to_index(location, my_map):
    """convert a index to a point"""
    x = location[0]
    y = location[1]
    width = my_map.info.width
    index = x + (y * width)
    return index