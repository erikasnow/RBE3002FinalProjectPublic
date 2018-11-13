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
  

def is_valid_loc(loc, my_map):
    """
        Gets if a point is a legal location
        :param loc: tuple of location
        :return: boolean is a legal point
    """


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


def point_to_index(location, my_map):
    """convert a index to a point"""
