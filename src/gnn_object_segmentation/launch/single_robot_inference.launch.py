from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # --- Launch Arguments ---
        DeclareLaunchArgument(
            'robot_name',
            default_value='rm03',
            description='Single robot for this instance. Run this launch file once per '
                        'robot, in separate terminals, to keep them isolated.'
        ),
        DeclareLaunchArgument(
            'robot_id',
            default_value='1',
            description='Numeric robot_id feature tag written into node_features. '
                        'Only matters if you need it to match training-time IDs.'
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