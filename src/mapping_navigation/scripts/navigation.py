#!/usr/bin/env python3

import rospy
import os
from geometry_msgs.msg import PoseStamped
from pure_pursuit import *
from std_msgs.msg import Float64
from path_helpers import *
from Models import *
from MapSerializerWindows import *
from map_information import *
from road_warning import *
from mapping_navigation.msg import PathData


class Navigation:
    """
    This does navigation
    """

    def __init__(self, path: List[Tuple[float, float, RoadSegmentType]], look_ahead_distance: float,
                 look_forward_gain: float, wheel_base: float):
        self.navigation_pub = rospy.Publisher('navigation', PathData, queue_size=10)
        self.last_index = 0
        self.target_index = 0
        self.path = []
        for x, y, segment in path:
            self.path.append(Point((x, y), segment))

        self.navigator = PurePursuit(look_ahead_distance, look_forward_gain, self.path)
        self.car_state = CarState(wheel_base)

    def listener(self):
        rospy.init_node('navigation', anonymous=True)
        rospy.Subscriber('airsimPose', PoseStamped, self.handle_gps_data)
        rospy.Subscriber("sensor/speed", Float64, self.handle_speed_data)
        rate = rospy.Rate(10)
        # Once to get initial starting index
        self.target_index = self.navigator.search_target_index(self.car_state)[0]

        while not rospy.is_shutdown():
            if len(self.path) - 1 > self.target_index:
                # Get the closest point to get the current road type.
                closest_point_index = self.find_nearest_point()
                # Look and see if a new segment is coming up ahead.
                next_segment = self.check_ahead(closest_point_index)

                nav_msg = PathData()
                nav_msg.steering_angle = self.get_steering_angle()
                nav_msg.current_segment = self.path[closest_point_index].segment_type.value
                nav_msg.next_segment = next_segment.value

                self.navigation_pub.publish(nav_msg)
            else:
                rospy.loginfo('Done route')
                self.navigation_pub.publish(PathData(0, 0, 3))
                # rospy.sigal_shutdown("Done route")
            rate.sleep()

    def handle_gps_data(self, position: PoseStamped):
        curr_point = Point((position.pose.position.x, position.pose.position.y))
        quaternion = (position.pose.orientation.x, position.pose.orientation.y,
                      position.pose.orientation.z, position.pose.orientation.w)
        self.car_state.update_pos(curr_point, quaternion)

    def handle_speed_data(self, speed: Float64):
        self.car_state.update_speed(speed.data)

    # Checks ahead up to 5 meters scanning for a new road segment.
    def check_ahead(self, closest_point) -> RoadWarning:
        for point in self.path[closest_point + 1:]:
            if point.segment_type != self.path[closest_point].segment_type and point.segment_type is not None:
                if self.car_state.calc_distance(point.coordinate[0], point.coordinate[1]) < 5:
                    rospy.loginfo(
                        f'The following road segment is 5 meters or less ahead {RoadWarning(point.segment_type.value)}')
                    return RoadWarning(point.segment_type.value)
            if self.car_state.calc_distance(point.coordinate[0], point.coordinate[1]) > 5:
                break

        return RoadWarning.SAME_AHEAD

    # Get the closet point to the car
    # Pure pursuit does not always keep track of the closest point so this is required to find it
    def find_nearest_point(self) -> int:
        dx = [self.car_state.rear_x - point.coordinate[0] for point in self.path]
        dy = [self.car_state.rear_y - point.coordinate[1] for point in self.path]
        distances = np.hypot(dx, dy)
        index = int(np.argmin(distances))
        return index

    """
    Returns a steering angle for the car in Airsim range of -1 to 1
    """

    def get_steering_angle(self) -> float:
        ackerman_angle_rad, target_index = self.navigator.pure_pursuit_steer_control(self.car_state)
        if self.target_index != target_index:
            self.last_index = self.target_index
            self.target_index = target_index
        ackerman_angle_ratio = math.degrees(ackerman_angle_rad) / 45
        rospy.loginfo(
            f"Target point is {self.path[self.target_index].coordinate[0]}, {self.path[self.target_index].coordinate[1]}")
        rospy.loginfo(f"car position {self.car_state.point.coordinate[0]}, {self.car_state.point.coordinate[1]}")
        rospy.loginfo(f'Current steering angle in degrees {math.degrees(ackerman_angle_rad)}')
        return ackerman_angle_ratio


# Can use either of the path methods by uncommenting one or the other if you want to test with the old path
if __name__ == "__main__":

    # map = load_from_file(os.path.abspath(os.path.dirname(__file__)) + "/new_coords.pickle")
    # # Assuming one path for the moment unless we plan on adding multiple per map in the future
    # created_path = map.convert_path(0)

    created_path = []
    old_path_format = read_points(os.path.abspath(os.path.dirname(__file__)) + "/coords.txt")
    for old_point in old_path_format:
        created_path.append((old_point.coordinate[0], old_point.coordinate[1], RoadSegmentType.STRAIGHT))

    navigation = Navigation(look_ahead_distance=3.0, look_forward_gain=0.1, path=created_path, wheel_base=2.2)
    navigation.listener()
