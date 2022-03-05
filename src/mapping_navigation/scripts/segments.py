#!/usr/bin/env python3
from map_information import RoadSegment, Point
from enum import Enum
from typing import Dict


class StraightSegment(RoadSegment):
    def __str__(self):
        pass


class TurnSegment(RoadSegment):
    def __str__(self):
        pass


class IntersectionSegment(RoadSegment):
    def __init__(self, intersection_outputs: Dict[int, Dict], center: Point):
        super().__init__()
        self.center = center
        self.intersection_outputs = intersection_outputs


class TurnEnum(Enum):
    FORWARD = 1
    LEFT = 2
    RIGHT = 3

