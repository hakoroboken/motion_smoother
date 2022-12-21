from launch import LaunchDescription , actions
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os.path

def generate_launch_description():
  return LaunchDescription([
    Node(
          package='motion_smoother',
            executable='motion_smoother',
            parameters=[{'linear.x_p' : 'true'} , 
                        {'linear.y_p' : 'true'} , 
                        {'linear.z_p' : 'true'} , 
                        {'angular.x_p' : 'true'} , 
                        {'angular.y_p' : 'true'} ,
                        {'angular.z_p' : 'true'}] ,
        ),
    ])
