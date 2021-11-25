#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory , JointTrajectoryPoint

from ament_index_python.packages import get_package_share_directory
import ikpy.chain
import sys 
import numpy as np

import os 
## Near to the ground to check grab
#2.1 0 1.94
## full strech right
#3.33 0.05 0.66
## Full strech up
#0.47 0 3.78
## Random
# 0.63 -0.148 2.39

class Trajectory_publisher4(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller4/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10);timer_period = 1
        self.timer= self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_8']
        
        package_share_dir = get_package_share_directory("line")
        urdf_file= os.path.join(package_share_dir, "urdf", "press1.urdf")
        
        ## Toolbox interface
        self.robot_initialize(urdf_file)
        
        
    
    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        global i
       
        if i <16:
            
            point.positions = [0.0]
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1 
        elif i< 17:
                   
            point.positions = [-0.5]
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1
        elif i< 18:
                   
            point.positions = [0.0]
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1
        else:
            i=0 
      
    def robot_initialize(self,urdf_file):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)
    




def main(args=None):
    global i
    i=0
    rclpy.init(args=args)
    joint_trajectory_object4 = Trajectory_publisher4()

    rclpy.spin(joint_trajectory_object4)
    joint_trajectory_object4.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
