import os
import xacro
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    package_name = 'scara_robot_description'
    xacro_name = "scara.xacro"
    scara_rviz_config_name='scara_rviz.rviz'
    
    ld = LaunchDescription()
    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{xacro_name}')
    doc=xacro.process_file(urdf_model_path)
    robot_desc=doc.toprettyxml(indent=' ')
    params = {'robot_description': robot_desc}
    scara_rviz_config_path=os.path.join(pkg_share, f'rviz/{scara_rviz_config_name}')
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[params]
        )

    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params]
        )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d',scara_rviz_config_path],
        output='screen'
        )

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz2_node)

    return ld
