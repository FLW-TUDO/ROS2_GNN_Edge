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

        # --- rosbag record ---
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record', '-o', bag_dir,
                'clock',
                '/tf', '/tf_static', 
                '/tracked_polygons', 'gnn_objects',
                '/graph_data', 
                '/navigate_to_pose/feedback', '/navigate_to_pose/result',
                #'/diagnostics',
                '/AS_3_neu/vicon_pose',
                '/AS_5_neu/vicon_pose',
                '/pallet_truck/vicon_pose',
                '/AS_1_neu/vicon_pose',
                '/AS_6_neu/vicon_pose',
                '/AS_4_neu/vicon_pose'
                # --- rm03 topics ---
            #     '/rm03/ti_mmwave/radar_scan_pcl', 'rm03/odom', '/rm03/vicon_pose',
            #     '/rm03/global_costmap/costmap_raw', '/rm03/plan', '/rm03/path', 
            #     '/rm03/cmd_vel', '/rm03/behavior_tree_log',
                
            #     # --- rm04 topics ---
            #     '/rm04/ti_mmwave/radar_scan_pcl', 'rm04/odom', '/rm04/vicon_pose',
            #     '/rm04/global_costmap/costmap_raw', '/rm04/plan', '/rm04/path', 
            #     '/rm04/cmd_vel', '/rm04/behavior_tree_log',

            #     # --- rm05 topics ---
            #     '/rm05/ti_mmwave/radar_scan_pcl', 'rm05/odom', '/rm05/vicon_pose',
            #     '/rm05/global_costmap/costmap_raw', '/rm05/plan', '/rm05/path', 
            #     '/rm05/cmd_vel', '/rm05/behavior_tree_log',

            #     # --- cr01 topics ---
            #     '/cr01/ti_mmwave/radar_scan_pcl', 'cr01/odom', '/cr01/vicon_pose',
            #     '/cr01/global_costmap/costmap_raw', '/cr01/plan', '/cr01/path', 
            #     '/cr01/cmd_vel', '/cr01/behavior_tree_log'
            # 
            ],
            output='screen'
        ),

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

# from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch.conditions import IfCondition
# import datetime
# import shlex


# VICON_TOPICS = [
#     '/AS_3_neu/vicon_pose',
#     '/AS_5_neu/vicon_pose',
#     '/pallet_truck/vicon_pose',
#     '/AS_1_neu/vicon_pose',
#     '/AS_6_neu/vicon_pose',
#     '/AS_4_neu/vicon_pose',
# ]

# # Keep these separate from Vicon topics.
# # These topics will always be passed to ros2 bag record.
# BASE_BAG_TOPICS = [
#     '/clock',
#     '/tf',
#     '/tf_static',
#     '/tracked_polygons',
#     'gnn_objects',
#     '/graph_data',
#     '/navigate_to_pose/feedback',
#     '/navigate_to_pose/result',
# ]


# def _bash_array(items):
#     return ' '.join(shlex.quote(item) for item in items)


# _CHECK_AND_RECORD = f"""\
# set -u

# BAG_ROOT="${{BAG_ROOT:-datalogging/rosbags}}"
# BAG_DIR="${{BAG_ROOT}}/${{RUN_ID}}/_${{BAG_TIMESTAMP}}_bag"

# mkdir -p "$(dirname "$BAG_DIR")"

# BASE_TOPICS=({_bash_array(BASE_BAG_TOPICS)})
# VICON_TOPICS=({_bash_array(VICON_TOPICS)})

# echo "[rosbag_guard] Bag output: $BAG_DIR"
# echo "[rosbag_guard] Waiting up to 10 s for optional Vicon topics..."

# MAX_WAIT=10
# ELAPSED=0
# INTERVAL=2

# # Wait until all Vicon topics appear, or timeout.
# # If only some appear, we will still record the available ones.
# while [ "$ELAPSED" -lt "$MAX_WAIT" ]; do
#     AVAILABLE="$(ros2 topic list 2>/dev/null || true)"

#     ALL_VICON_VISIBLE=true
#     for topic in "${{VICON_TOPICS[@]}}"; do
#         if ! printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic"; then
#             ALL_VICON_VISIBLE=false
#             break
#         fi
#     done

#     $ALL_VICON_VISIBLE && break

#     sleep "$INTERVAL"
#     ELAPSED=$((ELAPSED + INTERVAL))
# done

# AVAILABLE="$(ros2 topic list 2>/dev/null || true)"

# RECORD_TOPICS=("${{BASE_TOPICS[@]}}")
# ADDED_VICON=()
# MISSING_VICON=()

# for topic in "${{VICON_TOPICS[@]}}"; do
#     if printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic"; then
#         RECORD_TOPICS+=("$topic")
#         ADDED_VICON+=("$topic")
#     else
#         MISSING_VICON+=("$topic")
#     fi
# done

# echo "[rosbag_guard] Recording base topics:"
# printf '  %s\\n' "${{BASE_TOPICS[@]}}"

# if [ "${{#ADDED_VICON[@]}}" -gt 0 ]; then
#     echo "[rosbag_guard] Adding available Vicon topics:"
#     printf '  %s\\n' "${{ADDED_VICON[@]}}"
# else
#     echo "[rosbag_guard] No Vicon topics visible; continuing without Vicon topics."
# fi

# if [ "${{#MISSING_VICON[@]}}" -gt 0 ]; then
#     echo "[rosbag_guard] Skipping missing Vicon topics:"
#     printf '  %s\\n' "${{MISSING_VICON[@]}}"
# fi

# echo "[rosbag_guard] Starting rosbag record..."
# exec ros2 bag record -o "$BAG_DIR" "${{RECORD_TOPICS[@]}}"
# """


# def generate_launch_description():
#     timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'visualize',
#             default_value='False',
#             description='Enable RViz2 radar and robot position visualization'
#         ),
#         DeclareLaunchArgument(
#             'simulation',
#             default_value='False',
#             description='Use simulation mode'
#         ),
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='False',
#             description='Use simulation clock time if True'
#         ),
#         DeclareLaunchArgument(
#             'rviz_config',
#             default_value='src/gnn_object_segmentation/rviz/flw_hall_gnn.rviz',
#             description='Path to RViz config file'
#         ),
#         DeclareLaunchArgument(
#             'run_id',
#             default_value='default_run',
#             description='Run ID for log correlation'
#         ),
#         DeclareLaunchArgument(
#             'robot_list',
#             default_value='["rm03", "rm04", "rm05", "cr01"]',
#             description='List of robot namespaces passed as a JSON string'
#         ),
#         DeclareLaunchArgument(
#             'bag_root',
#             default_value='datalogging/rosbags',
#             description='Root directory for rosbag output'
#         ),

#         ExecuteProcess(
#             cmd=['bash', '-c', _CHECK_AND_RECORD],
#             additional_env={
#                 'RUN_ID': LaunchConfiguration('run_id'),
#                 'BAG_TIMESTAMP': TextSubstitution(text=timestamp),
#                 'BAG_ROOT': LaunchConfiguration('bag_root'),
#             },
#             output='screen',
#             emulate_tty=True,
#         ),

#         Node(
#             package='gnn_object_segmentation',
#             executable='data_merge_dynamic',
#             name='data_merge',
#             parameters=[
#                 {'run_id': LaunchConfiguration('run_id')},
#                 {'window_size': 5},
#                 {'robot_list': LaunchConfiguration('robot_list')},
#                 {'use_sim_time': LaunchConfiguration('use_sim_time')},
#             ],
#             output='screen',
#             arguments=[
#                 '--visualize', LaunchConfiguration('visualize'),
#                 '--simulation', LaunchConfiguration('simulation'),
#             ],
#         ),

#         Node(
#             package='gnn_object_segmentation',
#             executable='tracked_polygon_visualizer',
#             name='tracked_polygon_visualizer',
#             output='screen',
#             parameters=[{
#                 'input_topic': '/tracked_polygons',
#                 'output_topic': '/tracked_polygon_markers',
#                 'use_sim_time': LaunchConfiguration('use_sim_time'),
#             }],
#         ),

#         # Optional RViz2
#         # Node(
#         #     package='rviz2',
#         #     executable='rviz2',
#         #     name='rviz2',
#         #     output='screen',
#         #     arguments=['-d', LaunchConfiguration('rviz_config')],
#         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         #     condition=IfCondition(LaunchConfiguration('visualize')),
#         # ),
#     ])


# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument, ExecuteProcess
# # from launch_ros.actions import Node
# # from launch.substitutions import LaunchConfiguration, TextSubstitution
# # from launch.conditions import IfCondition
# # import datetime


# # VICON_TOPICS = [
# #     '/AS_3_neu/vicon_pose',
# #     '/AS_5_neu/vicon_pose',
# #     '/pallet_truck/vicon_pose',
# #     '/AS_1_neu/vicon_pose',
# #     '/AS_6_neu/vicon_pose',
# #     '/AS_4_neu/vicon_pose',
# # ]

# # BAG_TOPICS = [
# #     'clock',
# #     '/tf', '/tf_static',
# #     '/tracked_polygons', 'gnn_objects',
# #     '/graph_data',
# #     '/navigate_to_pose/feedback', '/navigate_to_pose/result',
# # ] + VICON_TOPICS

# # # Wait up to 10 s for all Vicon topics to appear in ros2 topic list,
# # # then start bag record (or skip if still missing).
# # # BAG_DIR is assembled inside bash from RUN_ID + BAG_TIMESTAMP env vars
# # # (PathJoinSubstitution is not reliably resolved in additional_env).
# # _CHECK_AND_RECORD = """\
# # BAG_DIR="datalogging/rosbags/${{RUN_ID}}/_${{BAG_TIMESTAMP}}_bag"
# # echo "[rosbag_guard] Checking Vicon topics (bag will go to $BAG_DIR)..."
# # MAX_WAIT=10; ELAPSED=0; INTERVAL=2; ALL_OK=false
# # while [ "$ELAPSED" -lt "$MAX_WAIT" ]; do
# #     AVAILABLE=$(ros2 topic list 2>/dev/null)
# #     ALL_OK=true
# #     for topic in {topics}; do
# #         printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic" || {{ ALL_OK=false; break; }}
# #     done
# #     $ALL_OK && break
# #     sleep $INTERVAL
# #     ELAPSED=$((ELAPSED + INTERVAL))
# # done
# # if ! $ALL_OK; then
# #     echo "[rosbag_guard] ABORT: not all Vicon topics visible after ${{MAX_WAIT}}s — bag record skipped"
# #     exit 0
# # fi
# # echo "[rosbag_guard] All Vicon topics OK — starting bag record"
# # exec ros2 bag record -o "$BAG_DIR" {bag_topics}
# # """.format(
# #     topics=' '.join(VICON_TOPICS),
# #     bag_topics=' '.join(BAG_TOPICS),
# # )


# # def generate_launch_description():
# #     timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

# #     return LaunchDescription([
# #         # --- Launch Arguments ---
# #         DeclareLaunchArgument(
# #             'visualize',
# #             default_value='False',
# #             description='Enable RViz2 radar and robot position visualization'
# #         ),
# #         DeclareLaunchArgument(
# #             'simulation',
# #             default_value='False',
# #             description='Use simulation mode (affects transform handling)'
# #         ),
# #         DeclareLaunchArgument(
# #             'use_sim_time',
# #             default_value='False',
# #             description='Use simulation (clock) time if True'
# #         ),
# #         DeclareLaunchArgument(
# #             'rviz_config',
# #             default_value='src/gnn_object_segmentation/rviz/flw_hall_gnn.rviz',
# #             description='Path to RViz config file'
# #         ),
# #         DeclareLaunchArgument(
# #             'run_id',
# #             default_value='default_run',
# #             description='Run ID for log correlation'
# #         ),
# #         DeclareLaunchArgument(
# #             'robot_list',
# #             default_value='["rm03", "rm04", "rm05", "cr01"]',
# #             description='List of robot namespaces passed as a JSON string'
# #         ),

# #         # --- Vicon guard + rosbag record (single process) ---
# #         # Waits up to 10 s for all Vicon topics, then records; skips silently if missing.
# #         # RUN_ID and BAG_TIMESTAMP are passed as simple substitutions; BAG_DIR is built in bash.
# #         ExecuteProcess(
# #             cmd=['bash', '-c', _CHECK_AND_RECORD],
# #             additional_env={
# #                 'RUN_ID': LaunchConfiguration('run_id'),
# #                 'BAG_TIMESTAMP': TextSubstitution(text=timestamp),
# #             },
# #             output='screen'
# #         ),

# #         # --- Dynamic Data Merger Node ---
# #         Node(
# #             package='gnn_object_segmentation',
# #             executable='data_merge_dynamic',
# #             name='data_merge',
# #             parameters=[
# #                 {"run_id": LaunchConfiguration('run_id')},
# #                 {"window_size": 5},
# #                 {"robot_list": LaunchConfiguration('robot_list')},
# #                 {"use_sim_time": LaunchConfiguration('use_sim_time')}
# #             ],
# #             output='screen',
# #             arguments=[from launch import LaunchDescription
# from launch.actions import DeclareLaunchArgument, ExecuteProcess
# from launch_ros.actions import Node
# from launch.substitutions import LaunchConfiguration, TextSubstitution
# from launch.conditions import IfCondition
# import datetime
# import shlex


# VICON_TOPICS = [
#     '/AS_3_neu/vicon_pose',
#     '/AS_5_neu/vicon_pose',
#     '/pallet_truck/vicon_pose',
#     '/AS_1_neu/vicon_pose',
#     '/AS_6_neu/vicon_pose',
#     '/AS_4_neu/vicon_pose',
# ]

# # Keep these separate from Vicon topics.
# # These topics will always be passed to ros2 bag record.
# BASE_BAG_TOPICS = [
#     '/clock',
#     '/tf',
#     '/tf_static',
#     '/tracked_polygons',
#     'gnn_objects',
#     '/graph_data',
#     '/navigate_to_pose/feedback',
#     '/navigate_to_pose/result',
# ]


# def _bash_array(items):
#     return ' '.join(shlex.quote(item) for item in items)


# _CHECK_AND_RECORD = f"""\
# set -u

# BAG_ROOT="${{BAG_ROOT:-datalogging/rosbags}}"
# BAG_DIR="${{BAG_ROOT}}/${{RUN_ID}}/_${{BAG_TIMESTAMP}}_bag"

# mkdir -p "$(dirname "$BAG_DIR")"

# BASE_TOPICS=({_bash_array(BASE_BAG_TOPICS)})
# VICON_TOPICS=({_bash_array(VICON_TOPICS)})

# echo "[rosbag_guard] Bag output: $BAG_DIR"
# echo "[rosbag_guard] Waiting up to 10 s for optional Vicon topics..."

# MAX_WAIT=10
# ELAPSED=0
# INTERVAL=2

# # Wait until all Vicon topics appear, or timeout.
# # If only some appear, we will still record the available ones.
# while [ "$ELAPSED" -lt "$MAX_WAIT" ]; do
#     AVAILABLE="$(ros2 topic list 2>/dev/null || true)"

#     ALL_VICON_VISIBLE=true
#     for topic in "${{VICON_TOPICS[@]}}"; do
#         if ! printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic"; then
#             ALL_VICON_VISIBLE=false
#             break
#         fi
#     done

#     $ALL_VICON_VISIBLE && break

#     sleep "$INTERVAL"
#     ELAPSED=$((ELAPSED + INTERVAL))
# done

# AVAILABLE="$(ros2 topic list 2>/dev/null || true)"

# RECORD_TOPICS=("${{BASE_TOPICS[@]}}")
# ADDED_VICON=()
# MISSING_VICON=()

# for topic in "${{VICON_TOPICS[@]}}"; do
#     if printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic"; then
#         RECORD_TOPICS+=("$topic")
#         ADDED_VICON+=("$topic")
#     else
#         MISSING_VICON+=("$topic")
#     fi
# done

# echo "[rosbag_guard] Recording base topics:"
# printf '  %s\\n' "${{BASE_TOPICS[@]}}"

# if [ "${{#ADDED_VICON[@]}}" -gt 0 ]; then
#     echo "[rosbag_guard] Adding available Vicon topics:"
#     printf '  %s\\n' "${{ADDED_VICON[@]}}"
# else
#     echo "[rosbag_guard] No Vicon topics visible; continuing without Vicon topics."
# fi

# if [ "${{#MISSING_VICON[@]}}" -gt 0 ]; then
#     echo "[rosbag_guard] Skipping missing Vicon topics:"
#     printf '  %s\\n' "${{MISSING_VICON[@]}}"
# fi

# echo "[rosbag_guard] Starting rosbag record..."
# exec ros2 bag record -o "$BAG_DIR" "${{RECORD_TOPICS[@]}}"
# """


# def generate_launch_description():
#     timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

#     return LaunchDescription([
#         DeclareLaunchArgument(
#             'visualize',
#             default_value='False',
#             description='Enable RViz2 radar and robot position visualization'
#         ),
#         DeclareLaunchArgument(
#             'simulation',
#             default_value='False',
#             description='Use simulation mode'
#         ),
#         DeclareLaunchArgument(
#             'use_sim_time',
#             default_value='False',
#             description='Use simulation clock time if True'
#         ),
#         DeclareLaunchArgument(
#             'rviz_config',
#             default_value='src/gnn_object_segmentation/rviz/flw_hall_gnn.rviz',
#             description='Path to RViz config file'
#         ),
#         DeclareLaunchArgument(
#             'run_id',
#             default_value='default_run',
#             description='Run ID for log correlation'
#         ),
#         DeclareLaunchArgument(
#             'robot_list',
#             default_value='["rm03", "rm04", "rm05", "cr01"]',
#             description='List of robot namespaces passed as a JSON string'
#         ),
#         DeclareLaunchArgument(
#             'bag_root',
#             default_value='datalogging/rosbags',
#             description='Root directory for rosbag output'
#         ),

#         ExecuteProcess(
#             cmd=['bash', '-c', _CHECK_AND_RECORD],
#             additional_env={
#                 'RUN_ID': LaunchConfiguration('run_id'),
#                 'BAG_TIMESTAMP': TextSubstitution(text=timestamp),
#                 'BAG_ROOT': LaunchConfiguration('bag_root'),
#             },
#             output='screen',
#             emulate_tty=True,
#         ),

#         Node(
#             package='gnn_object_segmentation',
#             executable='data_merge_dynamic',
#             name='data_merge',
#             parameters=[
#                 {'run_id': LaunchConfiguration('run_id')},
#                 {'window_size': 5},
#                 {'robot_list': LaunchConfiguration('robot_list')},
#                 {'use_sim_time': LaunchConfiguration('use_sim_time')},
#             ],
#             output='screen',
#             arguments=[
#                 '--visualize', LaunchConfiguration('visualize'),
#                 '--simulation', LaunchConfiguration('simulation'),
#             ],
#         ),

#         Node(
#             package='gnn_object_segmentation',
#             executable='tracked_polygon_visualizer',
#             name='tracked_polygon_visualizer',
#             output='screen',
#             parameters=[{
#                 'input_topic': '/tracked_polygons',
#                 'output_topic': '/tracked_polygon_markers',
#                 'use_sim_time': LaunchConfiguration('use_sim_time'),
#             }],
#         ),

#         # Optional RViz2
#         # Node(
#         #     package='rviz2',
#         #     executable='rviz2',
#         #     name='rviz2',
#         #     output='screen',
#         #     arguments=['-d', LaunchConfiguration('rviz_config')],
#         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
#         #     condition=IfCondition(LaunchConfiguration('visualize')),
#         # ),
#     ])


# # from launch import LaunchDescription
# # from launch.actions import DeclareLaunchArgument, ExecuteProcess
# # from launch_ros.actions import Node
# # from launch.substitutions import LaunchConfiguration, TextSubstitution
# # from launch.conditions import IfCondition
# # import datetime


# # VICON_TOPICS = [
# #     '/AS_3_neu/vicon_pose',
# #     '/AS_5_neu/vicon_pose',
# #     '/pallet_truck/vicon_pose',
# #     '/AS_1_neu/vicon_pose',
# #     '/AS_6_neu/vicon_pose',
# #     '/AS_4_neu/vicon_pose',
# # ]

# # BAG_TOPICS = [
# #     'clock',
# #     '/tf', '/tf_static',
# #     '/tracked_polygons', 'gnn_objects',
# #     '/graph_data',
# #     '/navigate_to_pose/feedback', '/navigate_to_pose/result',
# # ] + VICON_TOPICS

# # # Wait up to 10 s for all Vicon topics to appear in ros2 topic list,
# # # then start bag record (or skip if still missing).
# # # BAG_DIR is assembled inside bash from RUN_ID + BAG_TIMESTAMP env vars
# # # (PathJoinSubstitution is not reliably resolved in additional_env).
# # _CHECK_AND_RECORD = """\
# # BAG_DIR="datalogging/rosbags/${{RUN_ID}}/_${{BAG_TIMESTAMP}}_bag"
# # echo "[rosbag_guard] Checking Vicon topics (bag will go to $BAG_DIR)..."
# # MAX_WAIT=10; ELAPSED=0; INTERVAL=2; ALL_OK=false
# # while [ "$ELAPSED" -lt "$MAX_WAIT" ]; do
# #     AVAILABLE=$(ros2 topic list 2>/dev/null)
# #     ALL_OK=true
# #     for topic in {topics}; do
# #         printf '%s\\n' "$AVAILABLE" | grep -qxF "$topic" || {{ ALL_OK=false; break; }}
# #     done
# #     $ALL_OK && break
# #     sleep $INTERVAL
# #     ELAPSED=$((ELAPSED + INTERVAL))
# # done
# # if ! $ALL_OK; then
# #     echo "[rosbag_guard] ABORT: not all Vicon topics visible after ${{MAX_WAIT}}s — bag record skipped"
# #     exit 0
# # fi
# # echo "[rosbag_guard] All Vicon topics OK — starting bag record"
# # exec ros2 bag record -o "$BAG_DIR" {bag_topics}
# # """.format(
# #     topics=' '.join(VICON_TOPICS),
# #     bag_topics=' '.join(BAG_TOPICS),
# # )


# # def generate_launch_description():
# #     timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

# #     return LaunchDescription([
# #         # --- Launch Arguments ---
# #         DeclareLaunchArgument(
# #             'visualize',
# #             default_value='False',
# #             description='Enable RViz2 radar and robot position visualization'
# #         ),
# #         DeclareLaunchArgument(
# #             'simulation',
# #             default_value='False',
# #             description='Use simulation mode (affects transform handling)'
# #         ),
# #         DeclareLaunchArgument(
# #             'use_sim_time',
# #             default_value='False',
# #             description='Use simulation (clock) time if True'
# #         ),
# #         DeclareLaunchArgument(
# #             'rviz_config',
# #             default_value='src/gnn_object_segmentation/rviz/flw_hall_gnn.rviz',
# #             description='Path to RViz config file'
# #         ),
# #         DeclareLaunchArgument(
# #             'run_id',
# #             default_value='default_run',
# #             description='Run ID for log correlation'
# #         ),
# #         DeclareLaunchArgument(
# #             'robot_list',
# #             default_value='["rm03", "rm04", "rm05", "cr01"]',
# #             description='List of robot namespaces passed as a JSON string'
# #         ),

# #         # --- Vicon guard + rosbag record (single process) ---
# #         # Waits up to 10 s for all Vicon topics, then records; skips silently if missing.
# #         # RUN_ID and BAG_TIMESTAMP are passed as simple substitutions; BAG_DIR is built in bash.
# #         ExecuteProcess(
# #             cmd=['bash', '-c', _CHECK_AND_RECORD],
# #             additional_env={
# #                 'RUN_ID': LaunchConfiguration('run_id'),
# #                 'BAG_TIMESTAMP': TextSubstitution(text=timestamp),
# #             },
# #             output='screen'
# #         ),

# #         # --- Dynamic Data Merger Node ---
# #         Node(
# #             package='gnn_object_segmentation',
# #             executable='data_merge_dynamic',
# #             name='data_merge',
# #             parameters=[
# #                 {"run_id": LaunchConfiguration('run_id')},
# #                 {"window_size": 5},
# #                 {"robot_list": LaunchConfiguration('robot_list')},
# #                 {"use_sim_time": LaunchConfiguration('use_sim_time')}
# #             ],
# #             output='screen',
# #             arguments=[
# #                 '--visualize', LaunchConfiguration('visualize'),
# #                 '--simulation', LaunchConfiguration('simulation')
# #             ]
# #         ),

# #         # --- Tracked Polygon Marker Publisher ---
# #         Node(
# #             package='gnn_object_segmentation',
# #             executable='tracked_polygon_visualizer',
# #             name='tracked_polygon_visualizer',
# #             output='screen',
# #             parameters=[{
# #                 'input_topic': '/tracked_polygons',
# #                 'output_topic': '/tracked_polygon_markers',
# #                 'use_sim_time': LaunchConfiguration('use_sim_time')
# #             }]
# #         ),

# #         # --- Optional RViz2 and Arena Markers (Commented out by default) ---
# #         # Node(
# #         #     package='rviz2',
# #         #     executable='rviz2',
# #         #     name='rviz2',
# #         #     output='screen',
# #         #     arguments=['-d', LaunchConfiguration('rviz_config')],
# #         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
# #         #     condition=IfCondition(LaunchConfiguration('visualize'))
# #         # ),

# #         # Node(
# #         #     package='gnn_object_segmentation',
# #         #     executable='arena_marker_node',
# #         #     name='arena_marker_node',
# #         #     output='screen',
# #         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
# #         #     condition=IfCondition(LaunchConfiguration('visualize'))
# #         # ),
# #     ])

# #                 '--visualize', LaunchConfiguration('visualize'),
# #                 '--simulation', LaunchConfiguration('simulation')
# #             ]
# #         ),

# #         # --- Tracked Polygon Marker Publisher ---
# #         Node(
# #             package='gnn_object_segmentation',
# #             executable='tracked_polygon_visualizer',
# #             name='tracked_polygon_visualizer',
# #             output='screen',
# #             parameters=[{
# #                 'input_topic': '/tracked_polygons',
# #                 'output_topic': '/tracked_polygon_markers',
# #                 'use_sim_time': LaunchConfiguration('use_sim_time')
# #             }]
# #         ),

# #         # --- Optional RViz2 and Arena Markers (Commented out by default) ---
# #         # Node(
# #         #     package='rviz2',
# #         #     executable='rviz2',
# #         #     name='rviz2',
# #         #     output='screen',
# #         #     arguments=['-d', LaunchConfiguration('rviz_config')],
# #         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
# #         #     condition=IfCondition(LaunchConfiguration('visualize'))
# #         # ),

# #         # Node(
# #         #     package='gnn_object_segmentation',
# #         #     executable='arena_marker_node',
# #         #     name='arena_marker_node',
# #         #     output='screen',
# #         #     parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
# #         #     condition=IfCondition(LaunchConfiguration('visualize'))
# #         # ),
# #     ])
