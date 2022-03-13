from enum import Enum


# Status definition for the lane results
class LaneBoundStatus(Enum):
    TWO_BOUNDS = 1
    ONE_BOUND_LEFT = 2
    ONE_BOUND_RIGHT = 3
    NO_BOUNDS = 4
