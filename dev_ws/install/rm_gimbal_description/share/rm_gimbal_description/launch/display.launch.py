import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # 1. 获取包路径和 xacro 文件路径
    pkg_dir = get_package_share_directory('rm_gimbal_description')
    xacro_file = os.path.join(pkg_dir, 'urdf', 'rm_gimbal.urdf.xacro')  # 假设你的文件名为 rm_gimbal.urdf.xacro

    # 2. 将 xacro 文件转换为 urdf 文本
    doc = xacro.process_file(xacro_file)   # 返回的是 DOM
    robot_description_content = doc.toxml()  # 推荐 toxml()

    # 3. 配置 robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content}]
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
