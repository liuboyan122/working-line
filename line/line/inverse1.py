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

class Trajectory_publisher1(Node):

    def __init__(self):
        super().__init__('trajectory_publsiher_node')
        publish_topic = "/joint_trajectory_controller1/joint_trajectory"
        self.trajectory_publihser = self.create_publisher(JointTrajectory,publish_topic, 10);timer_period = 1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.joints = ['joint_1','joint_2','joint_4','joint_6','joint_7','joint_8','left_gripper_finger_joint','right_gripper_finger_joint']
        
        package_share_dir = get_package_share_directory("line")
        urdf_file= os.path.join(package_share_dir, "urdf", "arm1.urdf")
        
        ## Toolbox interface
        self.robot_initialize(urdf_file)
        
        
    
    def timer_callback(self):
        bazu_trajectory_msg = JointTrajectory()
        bazu_trajectory_msg.joint_names = self.joints
        point = JointTrajectoryPoint()
        global i
       
        if i <4:
            self.inverse_kinematics_solution(float(-1.25),float(0.0),float(0.3),"o" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=2)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1 
        elif i <6:
            self.inverse_kinematics_solution(float(-1.25),float(0.0),float(0.3),"a" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1 
        elif i <10:
            self.inverse_kinematics_solution(float(1.25),float(0.0),float(0.3),"a" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=2)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1    
        elif i <12:
            self.inverse_kinematics_solution(float(1.25),float(0.0),float(0.3),"o" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1  
        elif i <16:
            self.inverse_kinematics_solution(float(0.0),float(1.5),float(2.5),"o" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=2)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1  
        elif i <18:
            self.inverse_kinematics_solution(float(0.0),float(1.6),float(2.5),"o" )
            point.positions = self.goal_positions
            point.time_from_start = Duration(sec=1)
            bazu_trajectory_msg.points.append(point)
            self.trajectory_publihser.publish(bazu_trajectory_msg)
            print("\nTrajectory Sent !\n")
            i += 1  
        else:
          
            i=0 
      
    def robot_initialize(self,urdf_file):
        self.kuka_robot = ikpy.chain.Chain.from_urdf_file(urdf_file)
    

    def inverse_kinematics_solution(self,x,y,z,claw):
        angles=self.kuka_robot.inverse_kinematics([x,y,z])
        angles=np.delete(angles, [0,3,5,9,10])
        if (claw=="o"):
            print("\nClaw Open\n")
            self.goal_positions = list(np.append(angles ,[-0.01,-0.01]) )
        else:
            print("\nClaw Closed\n")
            self.goal_positions = list(np.append(angles ,[0.06, 0.06]) ) 
        print("\nInverse Kinematics Solution :\n" ,self.goal_positions)


def main(args=None):
    global i
    i=0
    rclpy.init(args=args)
    joint_trajectory_object1 = Trajectory_publisher1()

    rclpy.spin(joint_trajectory_object1)
    joint_trajectory_object1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
