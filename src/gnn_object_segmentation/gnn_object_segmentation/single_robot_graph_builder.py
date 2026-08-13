import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped
from collections import deque
import numpy as np
import math
import struct
from scipy.spatial.transform import Rotation as R
from sklearn.neighbors import NearestNeighbors
import pickle
from .visualizer import GraphVisualizer
import argparse
from gnn_interfaces.msg import GraphData
import tf2_ros
import os, csv
from datetime import datetime
import time


def pointcloud2_to_xyz_intensity(msg: PointCloud2):
    """Convert PointCloud2 message to an (N,4) numpy array of [x, y, z, intensity]."""
    points = []
    data = msg.data
    for i in range(msg.width):
        offset = i * msg.point_step
        x, = struct.unpack_from('<f', data, offset + 0)
        y, = struct.unpack_from('<f', data, offset + 4)
        z, = struct.unpack_from('<f', data, offset + 8)
        intensity, = struct.unpack_from('<f', data, offset + 16)
        points.append([x, y, z, intensity])
    return np.array(points, dtype=np.float32)


def statistical_outlier_removal(points, k=10, std_ratio=1.0):
    if len(points) < k:
        return points
    nbrs = NearestNeighbors(n_neighbors=k + 1).fit(points)
    distances, _ = nbrs.kneighbors(points)
    mean_distances = np.mean(distances[:, 1:], axis=1)
    global_mean = np.mean(mean_distances)
    global_std = np.std(mean_distances)
    threshold = global_mean + std_ratio * global_std
    mask = mean_distances < threshold
    return points[mask]


class SingleRobotGraphBuilder(Node):
    """
    Stage 1 node for a SINGLE robot only. Structurally cannot merge across
    robots: there is exactly one radar subscription, one pose subscription,
    one buffer pair, and the published graph's node features only ever
    contain points from this one robot. No robot_list, no per-robot loop,
    no cross-robot logic exists anywhere in this file.
    """

    def __init__(self, visualize=True, simulation=False):
        super().__init__('single_robot_graph_builder')

        self.simulation = simulation

        # --- Single-robot configuration ---
        self.declare_parameter("robot_name", "rm03")
        self.robot_name = self.get_parameter("robot_name").get_parameter_value().string_value

        # Fixed robot ID feature value. The GNN model expects a numeric
        # robot_id feature; for single-robot runs this is a constant tag,
        # not used to distinguish/merge multiple robots.
        self.declare_parameter("robot_id", 1)
        self.robot_id = self.get_parameter("robot_id").get_parameter_value().integer_value

        self.get_logger().info(
            f"Initialized SINGLE-robot Stage 1 for: {self.robot_name} (robot_id={self.robot_id})"
        )

        self.clock = self.get_clock()

        # Single buffers (no per-robot dict)
        self.pose_buffer = deque(maxlen=1000)
        self.radar_buffer = deque(maxlen=1000)
        self.last_radar_timestamp = 0.0
        self.last_vicon_timestamp = 0.0

        topic_pose = f'/{self.robot_name}/vicon_pose'
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            topic_pose,
            self.vicon_callback,
            10
        )

        topic_radar = f'/{self.robot_name}/ti_mmwave/radar_scan_pcl'
        self.radar_sub = self.create_subscription(
            PointCloud2,
            topic_radar,
            self.radar_callback,
            10
        )

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.03, self.process_and_publish)
        self.graph_pub = self.create_publisher(GraphData, '/graph_data', 10)

        self.merged_data_buffer = deque(maxlen=100)
        self.temporal_threshold = 2.0

        # Load Weights
        try:
            with open('normalization_weights_unified.pkl', 'rb') as f:
                self.norm_weights = pickle.load(f)
                print(f"✅ Norm weight has been loaded")
        except FileNotFoundError:
            self.get_logger().error(
                "normalization_weights_unified.pkl NOT FOUND. Please ensure it is in the working directory."
            )
            self.norm_weights = {}

        self.visualizer = GraphVisualizer(self, frame_id="map") if visualize else None

        # --- Parameters ---
        self.declare_parameter("run_id", "default_run")
        self.run_id = self.get_parameter("run_id").get_parameter_value().string_value

        self.declare_parameter("window_size", 3)
        self.N = self.get_parameter("window_size").get_parameter_value().integer_value

        # --- Logging ---
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(
            "datalogging/single_robot_graph_builder", f"{self.robot_name}_{self.run_id}_{timestamp}"
        )
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, "single_robot_graph_builder_log.csv")

        csv_header = [
            "timestamp_ros", "run_id", "robot_name", "window_size",
            "points", "node_count", "edge_count",
            "graph_build_time_ms", "merge_latency", "vicon_delay_ms"
        ]
        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)

    # --- Callbacks ---
    def radar_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points = pointcloud2_to_xyz_intensity(msg)

        self.radar_buffer.append((timestamp, points))
        self.radar_buffer = deque(
            [(ts, pts) for ts, pts in self.radar_buffer if timestamp - ts < 2.0],
            maxlen=1000
        )
        self.last_radar_timestamp = timestamp

    def vicon_callback(self, msg):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        pose = {
            'timestamp': timestamp,
            'translation': np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ]),
            'rotation': np.array([
                msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w
            ])
        }
        self.pose_buffer.append((timestamp, pose))
        self.last_vicon_timestamp = timestamp

    def get_closest(self, buffer, ref_time, max_diff=1.0):
        closest = None
        min_diff = float('inf')
        for ts, data in buffer:
            diff = abs(ts - ref_time)
            if diff < min_diff:
                min_diff = diff
            if diff < max_diff:
                closest = (ts, data)
        return closest

    def calculate_metrics(self, x, y, z, snr, bag_timestamp):
        range_val = math.sqrt(x * x + y * y + z * z)
        detectedAzimuth = 90.0 if x >= 0 else -90.0 if y == 0 else round(math.atan2(x, y) * 180 / math.pi, 3)
        detectedElevAngle = 90.0 if z >= 0 else -90.0 if (x == 0 and y == 0) else round(math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi, 3)

        return {
            'timestamp': bag_timestamp,
            'range': range_val,
            'azimuth': detectedAzimuth,
            'elevation': detectedElevAngle,
            'x': x, 'y': y, 'z': z, 'snr': snr,
            'robot_id': self.robot_id,
        }

    def process_radar_points(self, points, bag_timestamp):
        radar_points = []
        viz_points = [] if self.visualizer else None

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", f"{self.robot_name}/base_link",
                rclpy.time.Time(seconds=int(bag_timestamp), nanoseconds=int((bag_timestamp % 1) * 1e9))
            )
            translation = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            quaternion = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
        except Exception:
            return [], []

        rot_mat = R.from_quat(quaternion)

        for pt in points:
            x, y, z, snr = pt
            if (x < 0.3 and y <= 0.3):
                continue

            p_global = rot_mat.apply(np.array([x, y, z])) + translation
            xg, yg, zg = p_global

            if not (-12.0 <= xg <= 10.0 and -5.0 <= yg <= 7.0):
                continue

            point_data = self.calculate_metrics(xg, yg, zg, snr, bag_timestamp)
            radar_points.append(point_data)

            if self.visualizer:
                viz_points.append({'x': xg, 'y': yg, 'z': zg, 'robot_prefix_num': self.robot_name})

        return radar_points, viz_points

    def get_recent_window_frames(self, current_time):
        valid_frames = []
        for frame in reversed(self.merged_data_buffer):
            if abs(current_time - frame['timestamp']) <= self.temporal_threshold:
                valid_frames.append(frame)
            if len(valid_frames) == self.N:
                break
        if not valid_frames and self.merged_data_buffer:
            return [self.merged_data_buffer[-1]]
        return list(valid_frames)

    # --- Feature Construction ---
    def normalize_column(self, vals, params):
        method = params.get("method", "")
        if method == "minmax":
            return (vals - params["min"]) / (params["max"] - params["min"] + 1e-7)
        elif method == "zscore":
            return (vals - params["mean"]) / (params["std"] + 1e-7)
        return np.zeros_like(vals)

    def normalize_angle(self, vals, params):
        sin = np.sin(np.radians(vals))
        cos = np.cos(np.radians(vals))
        sin_norm = (sin - params.get("sin_mean", 0)) / (params.get("sin_std", 1) + 1e-7)
        cos_norm = (cos - params.get("cos_mean", 0)) / (params.get("cos_std", 1) + 1e-7)
        return sin_norm, cos_norm

    def build_edge_index_and_features(self, positions, snr_norm, timestamps, base_k=8):
        N = len(positions)
        k = min(base_k, N)
        if k < 1:
            return np.empty((2, 0)), np.empty((0, 5))

        nbrs = NearestNeighbors(n_neighbors=k, algorithm="ball_tree").fit(positions)
        distances, indices = nbrs.kneighbors(positions)

        edge_index = []
        edge_attr = []

        for i in range(N):
            for j in indices[i]:
                if i == j:
                    continue
                delta_pos = positions[j] - positions[i]
                delta_snr = snr_norm[j] - snr_norm[i]
                delta_t = timestamps[j] - timestamps[i]
                edge_index.append([i, j])
                edge_attr.append(np.hstack((delta_pos, delta_snr, delta_t)))

        if not edge_index:
            return np.empty((2, 0)), np.empty((0, 5))
        return np.array(edge_index, dtype=np.int64).T, np.array(edge_attr, dtype=np.float32)

    def build_graph_from_window(self, frames, norm_weights, base_k=8):
        all_points = []
        for frame in frames:
            ts = frame['timestamp']
            for pt in frame['radar_points']:
                pt = pt.copy()
                pt['timestamp'] = ts
                all_points.append(pt)

        if len(all_points) == 0:
            return None

        positions = np.array([[p['x'], p['y'], p['z']] for p in all_points])
        snr_vals = np.array([p['snr'] for p in all_points])
        range_vals = np.array([p['range'] for p in all_points])
        azimuth_vals = np.array([p['azimuth'] for p in all_points])
        elevation_vals = np.array([p['elevation'] for p in all_points])
        timestamps = np.array([p['timestamp'] for p in all_points])
        robot_ids = np.array([p['robot_id'] for p in all_points])

        snr_norm = self.normalize_column(snr_vals, norm_weights.get("snr", {}))
        range_norm = self.normalize_column(range_vals, norm_weights.get("range", {}))
        az_sin_norm, az_cos_norm = self.normalize_angle(azimuth_vals, norm_weights.get("azimuth", {}))
        el_sin_norm, el_cos_norm = self.normalize_angle(elevation_vals, norm_weights.get("elevation", {}))

        node_features_np = np.stack([
            positions[:, 0], positions[:, 1], positions[:, 2],
            snr_norm, range_norm,
            az_sin_norm, az_cos_norm,
            el_sin_norm, el_cos_norm,
            robot_ids
        ], axis=1)

        node_features = np.array(node_features_np, dtype=np.float32)
        edge_index, edge_attr = self.build_edge_index_and_features(positions, snr_norm, timestamps, base_k=base_k)

        return node_features, edge_index, edge_attr

    def publish_graph(self, node_features, edge_index, edge_attr):
        msg = GraphData()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.node_features = node_features.flatten().tolist()
        msg.node_feature_dim = node_features.shape[1] if node_features.ndim == 2 else 0
        msg.edge_index = edge_index.flatten().tolist()
        msg.edge_attr = edge_attr.flatten().tolist()
        msg.edge_attr_dim = edge_attr.shape[1] if (edge_attr.ndim == 2 and edge_attr.shape[0] > 0) else 0
        msg.num_nodes = node_features.shape[0]
        msg.num_edges = edge_index.shape[1] if edge_index.ndim == 2 else 0
        self.graph_pub.publish(msg)

    def prune_buffer(self, buffer, max_age, now):
        return deque(
            [(t, data) for (t, data) in buffer if now - t < max_age],
            maxlen=buffer.maxlen
        )

    def process_and_publish(self):
        merge_start_time = time.perf_counter()

        if len(self.radar_buffer) == 0:
            throttle_period = 5.0
            now_sec = self.clock.now().nanoseconds * 1e-9
            if not hasattr(self, 'last_no_data_log') or (now_sec - self.last_no_data_log > throttle_period):
                self.get_logger().warn(f"Waiting for data from {self.robot_name}...")
                self.last_no_data_log = now_sec
            return

        now = self.clock.now().nanoseconds * 1e-9
        fresh_threshold = 0.75

        latest_ts = self.radar_buffer[-1][0] if self.radar_buffer else 0
        if latest_ts == 0 or (now - latest_ts > fresh_threshold):
            age = now - latest_ts if latest_ts > 0 else float('inf')
            self.get_logger().warn(f"Data too old! Age: {age:.2f}s (Threshold: {fresh_threshold}s). Ignoring.")
            return

        ref_timestamp = latest_ts

        merged_data = {'timestamp': ref_timestamp, 'radar_points': []}

        radar_match = self.get_closest(self.radar_buffer, ref_timestamp)
        points_count = 0
        if radar_match:
            gnn_pts, viz_pts = self.process_radar_points(radar_match[1], radar_match[0])

            if len(gnn_pts) >= 15:
                pts_array = np.array([[pt['x'], pt['y'], pt['z']] for pt in gnn_pts])
                filtered = statistical_outlier_removal(pts_array, k=8, std_ratio=1.0)
                filtered_set = set(map(tuple, filtered))
                gnn_pts = [pt for pt in gnn_pts if (pt['x'], pt['y'], pt['z']) in filtered_set]

            merged_data['radar_points'] = gnn_pts
            points_count = len(gnn_pts)

        self.get_logger().info(f"[{self.robot_name}] Points processed: {points_count}")

        if points_count == 0:
            self.get_logger().warn(f"[{self.robot_name}] Zero points after processing (TF lookup failed?)")
            return

        self.merged_data_buffer.append(merged_data)

        graph_start_time = time.time()
        window_frames = self.get_recent_window_frames(merged_data['timestamp'])
        graph_data = self.build_graph_from_window(window_frames, self.norm_weights)
        graph_end_time = time.time()

        if graph_data:
            node_feats, edge_index, edge_attr = graph_data
            if edge_attr.shape[0] > 0 and edge_index.shape[1] > 0:
                self.publish_graph(node_feats, edge_index, edge_attr)

                vicon_delay = round((now - self.last_vicon_timestamp) * 1000, 2) if self.last_vicon_timestamp > 0 else 0.0

                with open(self.csv_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    row = [
                        now, self.run_id, self.robot_name, self.N, points_count,
                        node_feats.shape[0], edge_index.shape[1],
                        round((graph_end_time - graph_start_time) * 1000, 2),
                        (time.perf_counter() - merge_start_time) * 1000,
                        vicon_delay
                    ]
                    writer.writerow(row)

        # Cleanup
        max_buffer_age = 1.5
        self.radar_buffer = self.prune_buffer(self.radar_buffer, max_buffer_age, now)
        self.pose_buffer = self.prune_buffer(self.pose_buffer, max_buffer_age, now)
        self.merged_data_buffer = deque(
            [f for f in self.merged_data_buffer if now - f['timestamp'] < max_buffer_age],
            maxlen=self.merged_data_buffer.maxlen
        )


def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', default=True)
    parser.add_argument('--simulation', action='store_true', default=False)
    parsed_args, _ = parser.parse_known_args()

    node = SingleRobotGraphBuilder(visualize=parsed_args.visualize, simulation=parsed_args.simulation)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()