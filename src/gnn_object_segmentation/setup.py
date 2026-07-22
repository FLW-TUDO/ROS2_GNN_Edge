from setuptools import setup, find_packages

package_name = 'gnn_object_segmentation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    package_dir={'': '.'},
    install_requires=[
        'setuptools',
        'rclpy',
        'gnn_interfaces',  # Required to import custom msgs
    ],
    zip_safe=True,
    maintainer='yourname',
    maintainer_email='irfanfachrudin.priyanta@tu-dortmund.de',
    description='Radar GNN fusion node for object segmentation in a multi-robot system',
    license='Apache License 2.0',
    tests_require=['pytest'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_inference.launch.py']),
        ('share/' + package_name + '/launch', ['launch/multi_robot_inference_with_rosbag.launch.py']),    
        ('share/' + package_name + '/rviz', ['rviz/flw_hall.rviz']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_rm05.yaml']),
        ('share/' + package_name + '/config', ['config/waypoints_parp.yaml']),
        #('share/' + package_name + '/config', ['config/normalization_weights_unified.pkl']),
    ],
    entry_points={
        'console_scripts': [
            'gnn_fusion_inference_node = gnn_object_segmentation.gnn_fusion_inference_node:main',
            'data_merge = gnn_object_segmentation.data_merge:main',
            'data_merge_dynamic = gnn_object_segmentation.data_merge_dynamic:main',
            'arena_marker_node = gnn_object_segmentation.arena_boundary_publisher:main',
            'tracked_polygon_visualizer = gnn_object_segmentation.tracked_polygon_visualizer:main',
            'waypoints_publisher = gnn_object_segmentation.waypoints_publisher:main',
            # Add more nodes here as needed
        ],
    },
)
