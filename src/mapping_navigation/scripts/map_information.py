#!/usr/bin/env python3
from enum import Enum
from typing import Dict, Tuple, List


class DirEnum(Enum):
    NORTH = 1
    SOUTH = 2
    EAST = 3
    WEST = 4


class Point:
    def __init__(self, coordinate: Tuple[float, float], intersection_id: int = None):
        self.coordinate = coordinate
        self.intersection_id = intersection_id


class Lane:
    def __init__(self, points: List[Point]):
        self.points = points


class RoadSegment:
    def __init__(self, lanes: List[Lane], road_segment_id: int):
        self.lanes = lanes
        self.road_segment_id = road_segment_id


class Map:
    def __init__(self, roads: Dict[int, Dict[DirEnum, RoadSegment]] = {}):
        self.roads = roads
