#!/usr/bin/env python3
from enum import Enum


class RoadSegmentType(Enum):
    STRAIGHT = 0
    TURN = 1
    INTERSECTION = 2
