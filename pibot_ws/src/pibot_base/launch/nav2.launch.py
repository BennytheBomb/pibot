import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav2_params = os.path.join(
        get_package_share_directory('pibot_base'),
        'config',
        'nav2_params.yaml'
    )

    map_file = os.path.join(
        os.path.expanduser('~'),
        'new_map.yaml'
    )

    return LaunchDescription([
        # Map server — loads the saved map
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {'yaml_filename': map_file},
            ],
        ),

        # AMCL — localizes robot within the saved map
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params],
        ),

        # Nav2 lifecycle manager — manages map_server and amcl
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        ),

        # Planner server — global path planning
        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # Controller server — local path following
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params],
            remappings=[('cmd_vel', 'cmd_vel_nav')],
        ),

        # Smoother server
        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # Behavior server — recovery behaviors
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params],
        ),

        # BT navigator — high level navigation logic
        Node(
			package='nav2_bt_navigator',
			executable='bt_navigator',
			name='bt_navigator',
			output='screen',
			parameters=[
				nav2_params,
				{'default_nav_to_pose_bt_xml': '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml'},
				{'default_nav_through_poses_bt_xml': '/opt/ros/jazzy/share/nav2_bt_navigator/behavior_trees/navigate_w_replanning_only_if_path_becomes_invalid.xml'},
			],
		),

        # Waypoint follower
        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[nav2_params],
        ),

        # Velocity smoother
        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            parameters=[nav2_params],
            remappings=[
                ('cmd_vel', 'cmd_vel_nav'),
                ('cmd_vel_smoothed', 'cmd_vel'),
            ],
        ),

        # Nav2 lifecycle manager — manages navigation nodes
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'smoother_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother',
                ],
            }],
        ),
    ])
