#!/usr/bin/env python3
from typing import Optional
from map_information import RoadSegment


class Route:
    def __init__(self):
        self.road_segments = []
        self.current_road_index = 0

    def get_curr_segment(self) -> RoadSegment:
        return self.road_segments[self.current_road_index]

    def get_next_segment(self) -> Optional[RoadSegment]:
        if self.current_road_index == len(self.road_segments):
            return None
        return self.road_segments[self.current_road_index + 1]

    def next(self):
        self.current_road_index += 1

    def get_curr_lane(self):
        pass

    def get_next_lane(self):
        pass
