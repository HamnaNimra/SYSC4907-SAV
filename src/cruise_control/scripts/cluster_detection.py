import airsim
import numpy as np
import open3d as o3d
import os
import rospy
from geometry_msgs.msg import Point32


# Determine clusters in a point cloud, and derives bounding boxes from those point clouds
class ClusterDetection:
    def __init__(self):
        param_file = open(os.path.abspath(os.path.dirname(__file__)) + "/cluster_values.txt")
        self.min_num_points = int(param_file.readline())
        self.epsilon = float(param_file.readline())

        self.running_tests = False  # Set to true if running tests
        if self.running_tests:
            self.box_output_file = open(os.path.abspath(os.path.dirname(__file__)) + "/detected_boxes.txt", "w")

        host_ip = rospy.get_param('/host_ip')

        self.client = airsim.CarClient(ip=host_ip)
        self.client.confirmConnection()

    # Finds the bounding boxes of detected clusters in the point cloud. The returned
    # boxes are stored as a flat array in the with the box points in the following order:
    # [minX, minY, minZ, maxX, maxY, maxZ]
    def find_bounding_boxes(self, lidar_points: [Point32]) -> [float]:
        o3d_point_cloud = o3d.geometry.PointCloud()

        # Convert points with format used by point cloud to the format used by Open3D
        translated_points = []
        for point in lidar_points:
            if point.z > 0.1:
                translated_points.append([point.x, point.y, point.z])

        o3d_point_cloud.points = o3d.utility.Vector3dVector(translated_points)
        labels = np.array(o3d_point_cloud.cluster_dbscan(self.epsilon, self.min_num_points, print_progress=False))

        cluster_dic = dict()
        point_index = 0

        # Find what points belong to what cluster. Each index of labels corresponds to the point in the point cloud.
        # For example, label[0] refers to the cluster that the first point in the point cloud is a part of.
        for index in labels:

            # "-1" refers to not being a part of any cluster
            if index == -1:
                continue

            if index not in cluster_dic:
                cluster_dic[index] = []

            cluster_dic[index].append(translated_points[point_index])
            point_index += 1

        # Bounding boxes are stored as a flat array [minX, maxX, minY, maxY, minZ, maxZ).
        # Calculate the bounding boxes of the clusters
        bounding_boxes = []

        for cluster in cluster_dic.values():
            self.find_aabb(cluster, bounding_boxes)

        if self.running_tests:
            self.write_bounding_boxes_to_file(bounding_boxes)

        car_pos = self.client.simGetGroundTruthKinematics("").position
        bounding_boxes.append(car_pos.x_val.real)
        bounding_boxes.append(car_pos.y_val.real)
        bounding_boxes.append(car_pos.z_val.real)

        return bounding_boxes

    # Given a set of points, finds the enclosing bounding box of all the points.
    # The calculated box is appended to the result array input.
    def find_aabb(self, points: [float], result: [float]):

        # Max value of floats ensures that when iterating over points of the
        # bounding box, the points that comprise the box will be modified to
        # reflect the true min and max of the box
        min_x = 1e+308
        max_x = -1e+308

        min_y = 1e+308
        max_y = -1e+308

        min_z = 1e+308
        max_z = -1e+308

        for p in points:
            min_x = min(p[0], min_x)
            max_x = max(p[0], max_x)

            min_y = min(p[1], min_y)
            max_y = max(p[1], max_y)

            min_z = min(p[2], min_z)
            max_z = max(p[2], max_z)

        result.append(min_x)
        result.append(min_y)
        result.append(min_z)

        result.append(max_x)
        result.append(max_y)
        result.append(max_z)

    # Writes the passed in boxes to a file used by tests
    def write_bounding_boxes_to_file(self, boxes: [float]):

        car_pos = self.client.simGetGroundTruthKinematics("").position

        self.box_output_file.write("{},{},{},".format(car_pos.x_val.real, car_pos.y_val.real, car_pos.z_val.real))

        component_index = 0

        for box in boxes:
            if component_index == 0:
                box += car_pos.x_val.real
            elif component_index == 1:
                box += car_pos.y_val.real
            else:
                box += car_pos.z_val.real

            component_index += 1
            if component_index % 3 == 0:
                component_index = 0

            self.box_output_file.write("{},".format(box))

        self.box_output_file.write("\n")
        self.box_output_file.flush()
