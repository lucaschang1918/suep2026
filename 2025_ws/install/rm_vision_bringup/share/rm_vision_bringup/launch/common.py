import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.actions import Node
import xacro


launch_params = yaml.safe_load(open(os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'launch_params.yaml')))


pkg_dir = get_package_share_directory('rm_gimbal_description')
xacro_file = os.path.join(pkg_dir, 'urdf', 'rm_gimbal.urdf.xacro')
doc = xacro.process_file(xacro_file)
robot_description = doc.toxml()


robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[{'robot_description': robot_description,
                 'publish_frequency': 1000.0}]
)
node_params = os.path.join(
    get_package_share_directory('rm_vision_bringup'), 'config', 'node_params.yaml')

armor_tracker_node = Node(
    package='armor_tracker',
    executable='armor_tracker_node',
    output='both',
    emulate_tty=True,
    parameters=[node_params],
    ros_arguments=['--log-level', 'armor_tracker:='+launch_params['armor_tracker_log_level']],
)

# buff_tracker_node = Node(
#     package='buff_tracker',
#     executable='buff_tracker_node',
#     output='both',
#     emulate_tty=True,
#     parameters=[node_params],
#     ros_arguments=['--log-level', 'buff_tracker:='+launch_params['buff_tracker_log_level']],
# )

# auto_record_node = Node(
#     package='rm_auto_record',
#     executable='rm_auto_record_node',
#     output='both',
#     emulate_tty=True,
#     parameters=[node_params],
#     ros_arguments=['--log-level', 'rm_auto_record:='+launch_params['auto_record_log_level']],
# )