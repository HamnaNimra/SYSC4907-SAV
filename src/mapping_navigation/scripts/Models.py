#!/usr/bin/env python3

from enum import Enum
from typing import Optional, List, Dict, Tuple


# Model a point on the canvas
class Point:
    def __init__(self, x, y, pid):
        self.x = x
        self.y = y
        self.pid = pid

    def update_pid(self, pid):
        self.pid = pid

    def __str__(self):
        return f'{(self.x, self.y, self.pid)}'

    def __repr__(self):
        return self.__str__()


# Model a connection between two points
class Connection:
    def __init__(self, from_seg_id: int, from_point: Point, to_seg_id: int = None, to_point: Point = None):
        self.from_seg_id: int = from_seg_id
        self.to_seg_id: int = to_seg_id
        self.from_point: Point = from_point
        self.to_point: Point = to_point

    def __str__(self):
        return f'{self.from_point}->{self.to_point}, seg_id: {self.from_seg_id}->{self.to_seg_id}'

    def __repr__(self):
        return self.__str__()


class Lane:
    def __init__(self, points: List[Point] = None):
        self.points: List[Point] = points

    def __str__(self):
        return f'{self.points}'

    def __repr__(self):
        return f'{self.points}'

    def add_point(self, p: Point):
        self.points.append(p)

    def remove_point(self, point_id: int):
        # I hate list comprehensions
        self.points = [e for e in self.points if e.pid != point_id]


class RoadSegmentType(Enum):
    STRAIGHT = 0
    TURN = 1
    INTERSECTION = 2


class RoadSegment:
    def __init__(self, segment_id: int, seg_type: RoadSegmentType, lanes: List[Lane] = None):
        self.segment_id: int = segment_id
        self.seg_type: RoadSegmentType = seg_type
        self.lanes: List[Lane] = lanes

    def __str__(self):
        s = f'{self.seg_type}:'
        for lane in self.lanes:
            s = f'{s}\n{lane}'
        return s

    def __repr__(self):
        return self.__str__()

    def add_lane(self):
        self.lanes.append(Lane([]))

    def add_point(self, lane_id: int, point: Point):
        self.lanes[lane_id].add_point(point)

    # Delegate removal to the right Lane
    def remove_point(self, lane_id: int, point_id: int):
        self.lanes[lane_id].remove_point(point_id)


# Linear, deterministic path. Nothing dynamic.
class Path:
    def __init__(self):
        # First point has the Connection as None
        self.connections: List[Tuple[Connection, Point]] = []

    def __str__(self):
        if not self.connections:
            return 'empty path'
        else:
            s = ''
            for index, connection in enumerate(self.connections):
                s = f'{s}\n{index}) {connection[0]}'
            return s

    def __repr__(self):
        return self.__str__()

    # TODO: path validation?
    def add_to_path(self, connection: Connection, to_point: Point):
        self.connections.append((connection, to_point))

    def remove_last_point(self):
        if len(self.connections) > 0:
            del self.connections[-1]
        else:
            print('nothing to remove')

    def empty(self):
        return not self.connections


class MapModel:
    # Semantic versioning: Major.Minor.Patch
    version: str = '0.1.0'
    NH_scale_factor: float = 4.33564  # Calculated using collected points on the map
    NH_correction_factor: Tuple[int, int] = (-645, -573)

    def __init__(self):
        self.curr_id: int = 0
        # <segment id, segment>
        self.road_segments: Dict[int, RoadSegment] = {}
        # <segment id, connections[]>
        self.connections: Dict[int, List[Connection]] = {}
        self.paths: List[Path] = []
        self.instance_version: str = '0.1.0'

    def add_road_segment(self, seg_type: RoadSegmentType) -> int:
        self.road_segments[self.curr_id] = RoadSegment(self.curr_id, seg_type, [])
        self.curr_id += 1
        return self.curr_id - 1

    # Add empty path
    def add_path(self):
        self.paths.append(Path())

    def delete_path(self, index):
        del self.paths[index]

    # Use NH map correction factors. Implement other maps dynamically later?
    def convert_point(self, point: Point) -> Tuple[float, float]:
        # re-centering correction factor
        c1 = (point.x + MapModel.NH_correction_factor[0], point.y + MapModel.NH_correction_factor[1])
        # scale correction
        c2 = (c1[0]/MapModel.NH_scale_factor, c1[1]/MapModel.NH_scale_factor)
        # weird inversion correction due to the AirSim map being "upside-down"
        return -c2[1], c2[0]

    # Convert path to AirSim coords
    # TODO: include the segment type information somewhere...
    def convert_path(self, index) -> List[Tuple[float, float, RoadSegmentType]]:
        airsim_path: List[Tuple[float, float, RoadSegmentType]] = []
        sel_path = self.paths[index]
        # Empty path
        if sel_path.empty():
            pass

        # Iterate through points in the path
        for con, p in sel_path.connections:
            converted_point = self.convert_point(p)

            segment_type = RoadSegmentType.STRAIGHT
            if con.from_seg_id is not None:
                segment_type = self.road_segments[con.from_seg_id].seg_type
            airsim_path.append((converted_point[0], converted_point[1], segment_type))

        return airsim_path
