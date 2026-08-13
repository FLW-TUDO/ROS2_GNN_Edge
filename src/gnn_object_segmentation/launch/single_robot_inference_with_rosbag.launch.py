from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch.actions import ExecuteProcess
import datetime


def generate_launch_description():
    run_id = LaunchConfiguration('run_id')
    robot_name = LaunchConfiguration('robot_name')
    timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    # Bag path includes robot_name so two simultaneous instances (one per
    # terminal/robot) never write to the same directory.
    bag_dir = PathJoinSubstitution([
        TextSubstitution(text='datalogging/rosbags/'),
        run_id,
        TextSubstitution(text='_'),
        robot_name,
        TextSubstitution(text=f'_{timestamp}_bag')
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_name',
            default_value='rm03',
            description='Single robot for this instance. Run this launch file once per '
                        'robot, in separate terminals, to keep them isolated.'
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Numeric robot_id feature tag written into node_features.'
        ),
        DeclareLaunchArgument(
            'visualize',
            default_value='False',
            description='Enable radar and robot position visualization'
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
            'run_id',
            description='Waypoint config / run identifier (also used for log correlation) — '
                        'pass the actual run_id you use elsewhere, no default assumed here'
        ),
        DeclareLaunchArgument(
            'window_size',
            default_value='5',
            description='Temporal window size (frames), matches multi-robot default'
        ),

        # --- rosbag record ---
        # NOTE: /graph_data, /tracked_polygons, and gnn_objects are SHARED
        # topics (both single-robot instances and inference_node all use the
        # plain, un-namespaced topic names). If you run this launch file in
        # two terminals simultaneously, these topics in BOTH bags will
        # contain messages from BOTH robots, not just this one — filter by
        # the robot_id feature in node_features / contributor_ids during
        # post-processing if you need to isolate this robot's messages.
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', bag_dir,
                'clock',
                '/tf', '/tf_static',
                '/tracked_polygons', 'gnn_objects',
                '/graph_data',
                '/navigate_to_pose/feedback', '/navigate_to_pose/result',
                '/AS_3_neu/vicon_pose',
                '/AS_5_neu/vicon_pose',
                '/pallet_truck/vicon_pose',
                '/AS_1_neu/vicon_pose',
                '/AS_6_neu/vicon_pose',
                '/AS_4_neu/vicon_pose',
            ],
            output='screen'
        ),

        # --- Single-Robot Stage 1 Node ---
        Node(
            package='gnn_object_segmentation',
            executable='single_robot_graph_builder',
            name='single_robot_graph_builder',
            parameters=[
                {"run_id": LaunchConfiguration('run_id')},
                {"window_size": LaunchConfiguration('window_size')},
                {"robot_name": LaunchConfiguration('robot_name')},
                {"robot_id": LaunchConfiguration('robot_id')},
                {"use_sim_time": LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
            arguments=[
                '--visualize', LaunchConfiguration('visualize'),
                '--simulation', LaunchConfiguration('simulation')
            ]
        ),
    ])