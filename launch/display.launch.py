from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import LogInfo, ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', default_value='false',
            description='Use simulation clock if true'
        ),
        DeclareLaunchArgument(
            'xacro_file',
           # default_value='/my_robot_arm/urdf/asrs_robot.xacro',
            default_value=os.path.join(get_package_share_directory("my_robot_arm"), "urdf", "asrs_robot.xacro"),
            description='Path to the URDF xacro file'
            
        ),
        #DeclareLaunchArgument(
         #'xacro_file',
          #default_value='$(ros2 pkg prefix my_robot_arm)/share/my_robot_arm/urdf/asrs_robot.xacro'

        #),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', LaunchConfiguration('xacro_file')])
            }]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", os.path.join(get_package_share_directory("my_robot_arm"), "rviz", "rviz_config.rviz")]
        ),

#        Node(
#            package='joint_state_publisher',
#            executable='joint_state_publisher',
#            output='screen',
#            parameters=[{'robot_description': LaunchConfiguration('xacro_file')}]
#        ),

        Node(
            package='my_robot_arm',  # Change this to your package name
            executable='interactive_marker_publisher',  # Name of the new node
            name='interactive_marker_publisher_node',
            output='screen'
        ),
        
        Node(
            package='my_robot_arm',  # Replace with your package name
            executable='iksolver',       # Node executable name
            name='ik_solver_node',
            output='screen',
        ),

    ])
