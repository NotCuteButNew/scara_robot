import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import ExecuteProcess

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_name = 'scara_control'
    yaml_name = 'scara_control.yaml'
    xacro_name = "scara_control.xacro"
    package_path = get_package_share_directory(package_name)
    scara_control_dir = os.path.join(package_path, 'config', yaml_name)
    urdf_model_path=os.path.join(package_path,'urdf',xacro_name)
    doc = xacro.process_file(urdf_model_path)
    robot_desc = doc.toprettyxml(indent=' ')
    params = {'robot_description': robot_desc}

    ld = LaunchDescription()

    #启动gazebo
    start_gazebo_cmd = ExecuteProcess(cmd=[
        'gazebo', '--verbose', '-s', 'libgazebo_ros_init.so', '-s',
        'libgazebo_ros_factory.so'
    ],
                                      output='screen'),

    #加载scara
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'scara_robot', '-topic', '/robot_description'],
        output='screen',
        parameters=[params]),

    #加载控制器joint_state_broadcast
    joint_broad_spawner= Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcast'],
    ),
    
    #加载控制器joint_position_controller
    joint_position_spawner=Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_position_controller'],
    )
    
    ld.add_action(start_gazebo_cmd)
    ld.add_action(spawn_entity_cmd)
    ld.add_action(joint_broad_spawner)

    return ld
