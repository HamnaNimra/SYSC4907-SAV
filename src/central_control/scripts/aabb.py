from math import sqrt
from point import Point
from range import Range

class AABB:
    def __init__(self, min_point: Point, max_point: Point):
        self.x_range = Range(min_point.x, max_point.x)
        self.y_range = Range(min_point.y, max_point.y)
        self.z_range = Range(min_point.z, max_point.z)

    # Checks if there is an intersection between this AABB and the passed in AABB
    def intersection(self, other_aabb: 'AABB') -> bool:
        return self.x_range.overlaps(other_aabb.x_range) and \
               self.y_range.overlaps(other_aabb.y_range) and \
               self.z_range.overlaps(other_aabb.z_range)

    # Get the vector to the point that is closest to the passed in point.
    # Returns the distance to that point and the vector to it
    def vector_to_closest_point(self, pos: [float]) -> (float, [float]):
        aabb_corner_points = self.get_all_points()

        # Assume at the beginning that the closest point is extremely far away
        current_distance = 100000.0
        vector_to_point = [current_distance, current_distance, current_distance]

        # Indexes for each dimension of a points
        x = 0
        y = 1
        z = 2

        for point in aabb_corner_points:
            delta_x = pos[x] - point[x]
            delta_y = pos[y] - point[y]
            delta_z = pos[z] - point[z]

            distance_to_point = sqrt(delta_x ** 2 + delta_y ** 2 + delta_z ** 2)
            if distance_to_point < current_distance:
                vector_to_point = [delta_x, delta_y, delta_z]
                current_distance = distance_to_point

        return current_distance, vector_to_point

    # Get all eight corners of the AABB
    def get_all_points(self) -> [float]:
        return [
            [self.x_range.min, self.y_range.min, self.z_range.min],
            [self.x_range.min, self.y_range.min, self.z_range.max],
            [self.x_range.min, self.y_range.max, self.z_range.min],
            [self.x_range.min, self.y_range.max, self.z_range.max],

            [self.x_range.max, self.y_range.min, self.z_range.min],
            [self.x_range.max, self.y_range.min, self.z_range.max],
            [self.x_range.max, self.y_range.max, self.z_range.min],
            [self.x_range.max, self.y_range.max, self.z_range.max],
        ]

    def to_string(self):
        return "X-Range: {}, Y-Range: {}, Z-Range: {}".format(self.x_range.to_string(), self.y_range.to_string(), self.z_range.to_string())
