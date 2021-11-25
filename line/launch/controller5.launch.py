import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths

def generate_launch_description():
    package_share_dir = get_package_share_directory("line")
    urdf_file = os.path.join(package_share_dir, "urdf", "press2.urdf")
    
    controller_file = os.path.join(package_share_dir, "config", "jtc5.yaml")
    robot_description = {"robot_description": urdf_file}
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=["gazebo","-s","libgazebo_ros_factory.so",],
                output="screen",
            ),
            Node(
                package="gazebo_ros",
                node_executable="spawn_entity.py",
                arguments=["-entity","press2","-b","-file", urdf_file,'-x', '-1.25','-y', '0','-z', '0','-Y','0'],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                arguments=[urdf_file],
            ),
            
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                parameters=[robot_description, controller_file],
                output={
                    "stdout": "screen",
                    "stderr": "screen",
                },
            ),
            Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster5","controller_name","a5","--controller-manager", "/controller_manager"],
                ),

            Node(
                package="controller_manager",
                executable="spawner.py",
                arguments=["joint_trajectory_controller5", "-c", "/controller_manager","--controller-manager-timeout","1000"],
            ),
            Node(
                package="line",
                executable="inverse_kinematics5",
                
                
            ),
        ]
    )