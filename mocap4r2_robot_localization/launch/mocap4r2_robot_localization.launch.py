from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

package_name = "mocap4r2_robot_localization"

'''
Used to load parameters for composable nodes
'''
def dump_params(path, name):
    # Load the parameters specific to your ComposableNode
    with open(path, 'r') as file:
        return [yaml.safe_load(file)[name]['ros__parameters']]

def generate_launch_description():

  gtbody2robot_tf = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='gtbody2robot_tf',
    output='screen',
    arguments=["--x", "0", "--y", "0", "--z", "0.10", "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1", "--frame-id", "base_link", "--child-frame-id", "base_mocap"]
  )

  mocap4r2_robot_localization = Node(
    package=package_name,
    executable='localization_program',
    name='mocap4r2_robot_localization',
    output='screen',
    parameters=[os.path.join(get_package_share_directory(package_name), 'params', 'params.yaml')]
  )

  return LaunchDescription([gtbody2robot_tf, mocap4r2_robot_localization])
