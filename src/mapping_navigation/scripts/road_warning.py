#!/usr/bin/env python3
from enum import Enum


class RoadWarning(Enum):
    STRAIGHT_ROAD_AHEAD = 0
    TURN_AHEAD = 1
    INTERSECTION_AHEAD = 2
    SAME_AHEAD = 3
