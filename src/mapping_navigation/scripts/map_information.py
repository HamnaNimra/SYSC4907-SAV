#!/usr/bin/env python3
from enum import Enum

class Map:

    def __init__(self, roads):
        self.roads = {}

class Point:

    def __init__(self, coordinate, intersection_ID = None):
        self.coordinate = coordinate
        self.intersection_ID = intersection_ID

class RoadSegment:

    def __init__(self, lanes , road_segment_ID: int):
        self.lanes = lanes
        self.road_segment_ID = road_segment_ID

class Lane:

    def __init__(self, points):
        self.points = points

class DirEnum(Enum):
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4
