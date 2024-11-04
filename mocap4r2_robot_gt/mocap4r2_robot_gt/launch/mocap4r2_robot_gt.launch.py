from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

package_name = "mocap4r2_robot_gt"

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    # Load the parameters specific to your ComposableNode
    with open(path, 'r') as file:
        return [yaml.safe_load(file)[name]['ros__parameters']]

def generate_launch_description():

  mocap4r2_robot_gt = Node(
    package='mocap4r2_robot_gt',
    executable='gt_program',
    name='mocap4r2_gt',
    output='screen',
    parameters=[os.path.join(get_package_share_directory(package_name), 'params', 'params.yaml')]
  )

  return LaunchDescription([mocap4r2_robot_gt])
