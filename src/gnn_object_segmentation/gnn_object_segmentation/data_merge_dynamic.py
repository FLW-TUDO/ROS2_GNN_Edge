import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import Header
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
from rclpy.clock import Clock
from functools import partial  # <--- Essential for dynamic callbacks

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

class DataMergerNodeDynamic(Node):

    def __init__(self, visualize=True, simulation=False):
        super().__init__('data_merger_node')

        self.simulation = simulation
        
        # --- Dynamic Configuration ---
        ##self.declare_parameter("robot_list", ["rm04", "rm03"]) 
        ##self.robot_names = self.get_parameter("robot_list").get_parameter_value().string_array_value

        # ####
        # # --- Dynamic Configuration ---
        # # We declare it as a string because Launch files pass lists as JSON strings
        # self.declare_parameter("robot_list", '["rm04", "rm03"]') 
        
        # param_str = self.get_parameter("robot_list").get_parameter_value().string_value
        # try:
        #     # Parse the string '["rm04", "rm03"]' into a real Python list
        #     self.robot_names = json.loads(param_str)
        # except json.JSONDecodeError:
        #     self.get_logger().error(f"Failed to parse robot_list: {param_str}. Fallback to default.")
        #     self.robot_names = ["rm04", "rm03"]
        # #####

        # --- Dynamic Configuration ---
        # Declare as a list of strings (STRING_ARRAY) natively
        # Note the default value is a real list ["rm03", "rm04", "rm05", "cr01"], not a string '["..."]'
        self.declare_parameter("robot_list", ["rm03", "rm04", "rm05", "cr01"]) 
        
        # Retrieve directly as a string array
        self.robot_names = self.get_parameter("robot_list").get_parameter_value().string_array_value
        
        # Safety check: if for some reason it's empty, fallback
        if not self.robot_names:
            self.get_logger().warn("Received empty robot_list. Falling back to default.")
            self.robot_names = ["rm03", "rm04", "rm05", "cr01"]

        # Map robot names to numeric IDs (1, 2, 3...) based on their order in the list
        # IMPORTANT: The GNN .pt model might expect specific IDs (e.g. 1 and 2). 
        # Ensure your launch file passes the list in the correct order.
        self.robot_id_map = {name: i+1 for i, name in enumerate(self.robot_names)}
        
        self.get_logger().info(f"Initialized Dynamic Merger with robots: {self.robot_names}")
        self.get_logger().info(f"ID Mapping: {self.robot_id_map}")

        #self.clock = Clock()
        self.clock = self.get_clock() # (Uses ROS Sim Time)


        # Dynamic Buffers
        self.pose_buffer = {name: deque(maxlen=1000) for name in self.robot_names}
        self.radar_buffer = {name: deque(maxlen=1000) for name in self.robot_names}
        self.last_radar_timestamp = {name: 0.0 for name in self.robot_names}
        self.last_vicon_timestamp = {name: 0.0 for name in self.robot_names}

        # Dynamic Subscriptions
        self.pose_subs = []
        self.radar_subs = []

        for name in self.robot_names:
            # Vicon Pose Subscription
            topic_pose = f'/{name}/vicon_pose'
            sub_pose = self.create_subscription(
                PoseWithCovarianceStamped, 
                topic_pose, 
                partial(self.vicon_callback_generic, robot_name=name), 
                10
            )
            self.pose_subs.append(sub_pose)

            # Radar Subscription
            topic_radar = f'/{name}/ti_mmwave/radar_scan_pcl'
            sub_radar = self.create_subscription(
                PointCloud2, 
                topic_radar, 
                partial(self.radar_callback_generic, robot_name=name), 
                10
            )
            self.radar_subs.append(sub_radar)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_timer(0.03, self.sync_and_merge)
        self.graph_pub = self.create_publisher(GraphData, '/graph_data', 10)

        self.merged_data_buffer = deque(maxlen=100)
        self.temporal_threshold = 2.0

        # Load Weights
        try:
            with open('normalization_weights_unified.pkl', 'rb') as f:
                self.norm_weights = pickle.load(f)
                print(f"✅ Norm weight has been loaded")
        except FileNotFoundError:
            self.get_logger().error("normalization_weights_unified.pkl NOT FOUND. Please ensure it is in the working directory.")
            self.norm_weights = {}

        self.visualizer = GraphVisualizer(self, frame_id="map") if visualize else None

        # --- Parameters ---
        self.declare_parameter("run_id", "default_run")
        self.run_id = self.get_parameter("run_id").get_parameter_value().string_value

        self.declare_parameter("window_size", 3)
        self.N = self.get_parameter("window_size").get_parameter_value().integer_value

        # --- Logging ---
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join("datalogging/data_merge", f"{self.run_id}_{timestamp}")
        os.makedirs(self.log_dir, exist_ok=True)
        self.csv_path = os.path.join(self.log_dir, "data_merger_log.csv")

        # Dynamic CSV Header
        csv_header = ["timestamp_ros", "run_id", "window_size"]
        for name in self.robot_names:
            csv_header.append(f"{name}_points")
        csv_header.extend(["node_count", "edge_count", "inter_robot_edge_count", "graph_build_time_ms", "merge_latency"])
        for name in self.robot_names:
            csv_header.append(f"vicon_delay_{name}_ms")

        with open(self.csv_path, mode='w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(csv_header)

    # --- Generic Callbacks ---
    def radar_callback_generic(self, msg, robot_name):
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        points = pointcloud2_to_xyz_intensity(msg)
        
        self.radar_buffer[robot_name].append((timestamp, points))
        
        # Cleanup old data
        self.radar_buffer[robot_name] = deque(
            [(ts, pts) for ts, pts in self.radar_buffer[robot_name] if timestamp - ts < 2.0], 
            maxlen=1000
        )
        self.last_radar_timestamp[robot_name] = timestamp

    def vicon_callback_generic(self, msg, robot_name):
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
        self.pose_buffer[robot_name].append((timestamp, pose))
        self.last_vicon_timestamp[robot_name] = timestamp

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

    def calculate_metrics(self, x, y, z, snr, robot_name, bag_timestamp, robot_id):
        range_val = math.sqrt(x * x + y * y + z * z)
        detectedAzimuth = 90.0 if x >= 0 else -90.0 if y == 0 else round(math.atan2(x, y) * 180 / math.pi, 3)
        detectedElevAngle = 90.0 if z >= 0 else -90.0 if (x == 0 and y == 0) else round(math.atan2(z, math.sqrt(x * x + y * y)) * 180 / math.pi, 3)

        return {
            f'{robot_name}_timestamp': bag_timestamp,
            'range': range_val,
            'azimuth': detectedAzimuth,
            'elevation': detectedElevAngle,
            'x': x, 'y': y, 'z': z, 'snr': snr,
            'robot_id': robot_id,
            'robot_prefix_num': robot_name,
        }

    def process_radar_points(self, points, robot_name, bag_timestamp, robot_id):
        radar_points = []
        viz_points = [] if self.visualizer else None

        try:
            tf = self.tf_buffer.lookup_transform(
                "map", f"{robot_name}/base_link",
                rclpy.time.Time(seconds=int(bag_timestamp), nanoseconds=int((bag_timestamp % 1) * 1e9))
            )
            translation = np.array([tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z])
            quaternion = np.array([tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w])
        except Exception:
            # self.get_logger().warn(f"[TF] Failed to get transform for {robot_name}")
            return [], []

        rot_mat = R.from_quat(quaternion)

        for pt in points:
            x, y, z, snr = pt
            # Filter close to robot
            if (x < 0.3 and y <= 0.3): continue
            
            # Local to Global
            p_global = rot_mat.apply(np.array([x, y, z])) + translation
            xg, yg, zg = p_global

            # Filter bounds
            if not (-12.0 <= xg <= 10.0 and -5.0 <= yg <= 7.0): continue

            point_data = self.calculate_metrics(xg, yg, zg, snr, robot_name, bag_timestamp, robot_id)
            radar_points.append(point_data)

            if self.visualizer:
                viz_points.append({'x': xg, 'y': yg, 'z': zg, 'robot_prefix_num': robot_name})

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
        if k < 1: return np.empty((2, 0)), np.empty((0, 5))

        nbrs = NearestNeighbors(n_neighbors=k, algorithm="ball_tree").fit(positions)
        distances, indices = nbrs.kneighbors(positions)

        edge_index = []
        edge_attr = []

        for i in range(N):
            for j in indices[i]:
                if i == j: continue
                delta_pos = positions[j] - positions[i]
                delta_snr = snr_norm[j] - snr_norm[i]
                delta_t = timestamps[j] - timestamps[i]
                edge_index.append([i, j])
                edge_attr.append(np.hstack((delta_pos, delta_snr, delta_t)))

        if not edge_index: return np.empty((2, 0)), np.empty((0, 5))
        return np.array(edge_index, dtype=np.int64).T, np.array(edge_attr, dtype=np.float32)

    def build_graph_from_window(self, frames, norm_weights, base_k=8):
        all_points = []
        for frame in frames:
            ts = frame['timestamp']
            for radar_source, radar_pts in frame['radar_points'].items():
                if not radar_pts: continue
                for pt in radar_pts:
                    pt = pt.copy()
                    pt['timestamp'] = ts
                    all_points.append(pt)
        
        if len(all_points) == 0: return None

        positions = np.array([[p['x'], p['y'], p['z']] for p in all_points])
        snr_vals = np.array([p['snr'] for p in all_points])
        range_vals = np.array([p['range'] for p in all_points])
        azimuth_vals = np.array([p['azimuth'] for p in all_points])
        elevation_vals = np.array([p['elevation'] for p in all_points])
        timestamps = np.array([p['timestamp'] for p in all_points])
        
        # Dynamic ID extraction using the map created in __init__
        robot_ids = np.array([self.robot_id_map.get(p['robot_prefix_num'], 0) for p in all_points])

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

    def prune_buffer(self, buffer_dict, max_age, now):
        for key in buffer_dict:
            buffer_dict[key] = [(t, data) for (t, data) in buffer_dict[key] if now - t < max_age]


    def sync_and_merge(self):
        merge_start_time = time.perf_counter()
        
        # DEBUG 1: Check Buffers
        # Create a quick summary of buffer sizes
        buffer_sizes = {name: len(self.radar_buffer[name]) for name in self.robot_names}
        
        # Check if ANY robot has data
        has_data = any(size > 0 for size in buffer_sizes.values())
        if not has_data:
            # Only print this once every 5 seconds to avoid spamming if truly empty
            throttle_period = 5.0 
            now_sec = self.clock.now().nanoseconds * 1e-9
            if not hasattr(self, 'last_no_data_log') or (now_sec - self.last_no_data_log > throttle_period):
                self.get_logger().warn(f"Waiting for data... Buffers: {buffer_sizes}")
                self.last_no_data_log = now_sec
            return

        now = self.clock.now().nanoseconds * 1e-9
        fresh_threshold = 0.75 

        # Determine reference timestamp (latest from any robot)
        latest_timestamps = [
            (self.radar_buffer[name][-1][0] if self.radar_buffer[name] else 0) 
            for name in self.robot_names
        ]
        
        # DEBUG 2: Check Time Freshness
        # If all data is old, skip
        if all((now - ts > fresh_threshold) for ts in latest_timestamps if ts > 0):
            # Calculate how old the data is
            age = max([now - ts for ts in latest_timestamps if ts > 0], default=0)
            self.get_logger().warn(f"Data too old! Age: {age:.2f}s (Threshold: {fresh_threshold}s). Ignoring.")
            return

        # Pick the latest available timestamp as reference
        ref_timestamp = max(latest_timestamps)
        
        merged_data = {'timestamp': ref_timestamp, 'poses': {}, 'radar_points': {}}
        points_counts = []

        # Iterate all robots and sync
        for name in self.robot_names:
            radar_match = self.get_closest(self.radar_buffer[name], ref_timestamp)
            if radar_match:
                # DEBUG 3: Inside process_radar_points, there is a try/except for TF.
                # If that fails, it returns empty lists.
                gnn_pts, viz_pts = self.process_radar_points(
                    radar_match[1], name, radar_match[0], self.robot_id_map[name]
                )
                
                # Statistical Outlier Removal
                if len(gnn_pts) >= 15:
                    pts_array = np.array([[pt['x'], pt['y'], pt['z']] for pt in gnn_pts])
                    filtered = statistical_outlier_removal(pts_array, k=8, std_ratio=1.0)
                    filtered_set = set(map(tuple, filtered))
                    gnn_pts = [pt for pt in gnn_pts if (pt['x'], pt['y'], pt['z']) in filtered_set]

                merged_data['radar_points'][name] = gnn_pts
                points_counts.append(len(gnn_pts))
            else:
                points_counts.append(0)

        # --- THIS IS THE MISSING LOG FROM THE ORIGINAL SCRIPT ---
        # It prints how many points were actually merged for each robot
        points_summary = dict(zip(self.robot_names, points_counts))
        self.get_logger().info(f"[SYNC] Points merged: {points_summary}")

        # Skip if totally empty
        if sum(points_counts) == 0:
            self.get_logger().warn("[SYNC] Zero points merged after processing (TF lookup failed?)")
            return

        self.merged_data_buffer.append(merged_data)
        
        # Build Graph
        graph_start_time = time.time()
        window_frames = self.get_recent_window_frames(merged_data['timestamp'])
        graph_data = self.build_graph_from_window(window_frames, self.norm_weights)
        graph_end_time = time.time()

        if graph_data:
            node_feats, edge_index, edge_attr = graph_data
            if edge_attr.shape[0] > 0 and edge_index.shape[1] > 0:
                self.publish_graph(node_feats, edge_index, edge_attr)
                
                # Log stats
                inter_robot_edges = int(np.sum([
                    1 for i, j in edge_index.T if node_feats[i, -1] != node_feats[j, -1]
                ]))
                
                # Calculate delays dynamically
                vicon_delays = []
                for name in self.robot_names:
                    last_ts = self.last_vicon_timestamp[name]
                    vicon_delays.append(round((now - last_ts) * 1000, 2) if last_ts > 0 else 0.0)

                with open(self.csv_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    row = [now, self.run_id, self.N] + points_counts + [
                        node_feats.shape[0], edge_index.shape[1], inter_robot_edges,
                        round((graph_end_time - graph_start_time) * 1000, 2),
                        (time.perf_counter() - merge_start_time) * 1000
                    ] + vicon_delays
                    writer.writerow(row)

        # Cleanup
        max_buffer_age = 1.5
        
        self.prune_buffer(self.radar_buffer, max_buffer_age, now)
        self.prune_buffer(self.pose_buffer, max_buffer_age, now)
        
        self.merged_data_buffer = [f for f in self.merged_data_buffer if now - f['timestamp'] < max_buffer_age]
""" 
    def sync_and_merge(self):
        merge_start_time = time.perf_counter()
        
        # Check if ANY robot has data
        has_data = any(len(self.radar_buffer[name]) > 0 for name in self.robot_names)
        if not has_data:
            return

        now = self.clock.now().nanoseconds * 1e-9
        fresh_threshold = 0.75 

        # Determine reference timestamp (latest from any robot)
        latest_timestamps = [
            (self.radar_buffer[name][-1][0] if self.radar_buffer[name] else 0) 
            for name in self.robot_names
        ]
        
        # If all data is old, skip
        if all((now - ts > fresh_threshold) for ts in latest_timestamps if ts > 0):
            return

        # Pick the latest available timestamp as reference
        ref_timestamp = max(latest_timestamps)
        
        merged_data = {'timestamp': ref_timestamp, 'poses': {}, 'radar_points': {}}
        points_counts = []

        # Iterate all robots and sync
        for name in self.robot_names:
            radar_match = self.get_closest(self.radar_buffer[name], ref_timestamp)
            if radar_match:
                gnn_pts, viz_pts = self.process_radar_points(
                    radar_match[1], name, radar_match[0], self.robot_id_map[name]
                )
                
                # Statistical Outlier Removal
                if len(gnn_pts) >= 15:
                    pts_array = np.array([[pt['x'], pt['y'], pt['z']] for pt in gnn_pts])
                    filtered = statistical_outlier_removal(pts_array, k=8, std_ratio=1.0)
                    # Simple set filtering (careful with float precision)
                    # A more robust way is to use indices mask from SOR if possible, 
                    # but here we stick to the original logic for simplicity:
                    filtered_set = set(map(tuple, filtered))
                    gnn_pts = [pt for pt in gnn_pts if (pt['x'], pt['y'], pt['z']) in filtered_set]

                merged_data['radar_points'][name] = gnn_pts
                points_counts.append(len(gnn_pts))
            else:
                points_counts.append(0)

        # Skip if totally empty
        if sum(points_counts) == 0:
            return

        self.merged_data_buffer.append(merged_data)
        
        # Build Graph
        graph_start_time = time.time()
        window_frames = self.get_recent_window_frames(merged_data['timestamp'])
        graph_data = self.build_graph_from_window(window_frames, self.norm_weights)
        graph_end_time = time.time()

        if graph_data:
            node_feats, edge_index, edge_attr = graph_data
            if edge_attr.shape[0] > 0 and edge_index.shape[1] > 0:
                self.publish_graph(node_feats, edge_index, edge_attr)
                
                # Log stats
                inter_robot_edges = int(np.sum([
                    1 for i, j in edge_index.T if node_feats[i, -1] != node_feats[j, -1]
                ]))
                
                # Calculate delays dynamically
                vicon_delays = []
                for name in self.robot_names:
                    last_ts = self.last_vicon_timestamp[name]
                    vicon_delays.append(round((now - last_ts) * 1000, 2) if last_ts > 0 else 0.0)

                with open(self.csv_path, mode='a', newline='') as file:
                    writer = csv.writer(file)
                    row = [now, self.run_id, self.N] + points_counts + [
                        node_feats.shape[0], edge_index.shape[1], inter_robot_edges,
                        round((graph_end_time - graph_start_time) * 1000, 2),
                        (time.perf_counter() - merge_start_time) * 1000
                    ] + vicon_delays
                    writer.writerow(row)

        # # Cleanup
        # max_buffer_age = 1.5
        # for name in self.robot_names:
        #     self.prune_buffer(self.radar_buffer[name], max_buffer_age, now)
        #     self.prune_buffer(self.pose_buffer[name], max_buffer_age, now)
        
        # self.merged_data_buffer = [f for f in self.merged_data_buffer if now - f['timestamp'] < max_buffer_age]

        # Cleanup
        max_buffer_age = 1.5
        
        # FIX: Pass the WHOLE dictionary. Do NOT loop through names here.
        # The prune_buffer function already contains a loop to handle all keys.
        self.prune_buffer(self.radar_buffer, max_buffer_age, now)
        self.prune_buffer(self.pose_buffer, max_buffer_age, now)
        
        self.merged_data_buffer = [f for f in self.merged_data_buffer if now - f['timestamp'] < max_buffer_age]
 """
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser()
    parser.add_argument('--visualize', action='store_true', default=True)
    parser.add_argument('--simulation', action='store_true', default=False)
    parsed_args, _ = parser.parse_known_args()

    node = DataMergerNodeDynamic(visualize=parsed_args.visualize, simulation=parsed_args.simulation)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()