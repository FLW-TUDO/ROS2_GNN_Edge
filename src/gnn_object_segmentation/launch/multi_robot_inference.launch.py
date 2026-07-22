from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.conditions import IfCondition
from launch.actions import ExecuteProcess
import datetime


def generate_launch_description():
    # Generate timestamp at launch time
    run_id = LaunchConfiguration('run_id')
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    bag_dir = PathJoinSubstitution([
        TextSubstitution(text='datalogging/rosbags/'),
        run_id,
        TextSubstitution(text=f'_{timestamp}_bag')
    ])

    return LaunchDescription([
        # --- Launch Arguments ---
        DeclareLaunchArgument(
            'visualize',
            default_value='False',
            description='Enable RViz2 radar and robot position visualization'
        ),
        DeclareLaunchArgument(
            'simulation',
            default_value='False',
            description='Use simulation mode (affects transform handling)'
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='False',
            description='Use simulation (clock) time if True'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value='src/gnn_object_segmentation/rviz/flw_hall_gnn.rviz',
            description='Path to RViz config file'
        ),
        DeclareLaunchArgument(
            'run_id',
            default_value='default_run',
            description='Run ID for log correlation'
        ),
        # --- Updated Default Robot List ---
        DeclareLaunchArgument(
            'robot_list',
            default_value='["rm03", "rm04", "rm05", "cr01"]',
            description='List of robot namespaces passed as a JSON string'
        ),

        # # --- rosbag record ---
        # ExecuteProcess(
        #     cmd=[
        #         'ros2', 'bag', 'record', '-o', bag_dir,
        #         'clock',
        #         '/tf', '/tf_static', 
        #         '/tracked_polygons', 'gnn_objects',
        #         '/graph_data', 
        #         '/navigate_to_pose/feedback', '/navigate_to_pose/result',
                
        #         # --- rm03 topics ---
        #         '/rm03/ti_mmwave/radar_scan_pcl', 'rm03/odom', '/rm03/vicon_pose',
        #         '/rm03/global_costmap/costmap_raw', '/rm03/plan', '/rm03/path', 
        #         '/rm03/cmd_vel', '/rm03/behavior_tree_log',
                
        #         # --- rm04 topics ---
        #         '/rm04/ti_mmwave/radar_scan_pcl', 'rm04/odom', '/rm04/vicon_pose',
        #         '/rm04/global_costmap/costmap_raw', '/rm04/plan', '/rm04/path', 
        #         '/rm04/cmd_vel', '/rm04/behavior_tree_log',

        #         # --- rm05 topics ---
        #         '/rm05/ti_mmwave/radar_scan_pcl', 'rm05/odom', '/rm05/vicon_pose',
        #         '/rm05/global_costmap/costmap_raw', '/rm05/plan', '/rm05/path', 
        #         '/rm05/cmd_vel', '/rm05/behavior_tree_log',

        #         # --- cr01 topics ---
        #         '/cr01/ti_mmwave/radar_scan_pcl', 'cr01/odom', '/cr01/vicon_pose',
        #         '/cr01/global_costmap/costmap_raw', '/cr01/plan', '/cr01/path', 
        #         '/cr01/cmd_vel', '/cr01/behavior_tree_log'
        #     ],
        #     output='screen'
        # ),

        # --- Dynamic Data Merger Node ---
        Node(
            package='gnn_object_segmentation',
            executable='data_merge_dynamic', 
            name='data_merge',
            parameters=[
                {"run_id": LaunchConfiguration('run_id')},
                {"window_size": 5},
                {"robot_list": LaunchConfiguration('robot_list')},
                {"use_sim_time": LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
            arguments=[
                '--visualize', LaunchConfiguration('visualize'),
                '--simulation', LaunchConfiguration('simulation')
            ]
        ),

        # --- Tracked Polygon Marker Publisher ---
        Node(
            package='gnn_object_segmentation',
            executable='tracked_polygon_visualizer',
            name='tracked_polygon_visualizer',
            output='screen',
            parameters=[{
                'input_topic': '/tracked_polygons',
                'output_topic': '/tracked_polygon_markers',
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),

        # --- Optional RViz2 and Arena Markers (Commented out by default) ---
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     output='screen',
        #     arguments=['-d', LaunchConfiguration('rviz_config')],
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     condition=IfCondition(LaunchConfiguration('visualize'))
        # ),

        # Node(
        #     package='gnn_object_segmentation',
        #     executable='arena_marker_node',
        #     name='arena_marker_node',
        #     output='screen',
        #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        #     condition=IfCondition(LaunchConfiguration('visualize'))
        # ),
    ])