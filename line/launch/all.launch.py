import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from scripts import GazeboRosPaths
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
def generate_launch_description():
    

    return LaunchDescription(
        [   
        
        IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/controller1.launch.py']),),
      
        IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/controller2.launch.py']),),

            IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/controller3.launch.py']),),
      
        IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/controller4.launch.py']),),

  IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/controller5.launch.py']),),
      
        IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/gazebo1.launch.py']),),

       IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('line'), 'launch'),
         '/gazebo2.launch.py']),),


        ]
    )