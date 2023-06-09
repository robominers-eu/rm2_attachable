import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
use_sim_time = True

def generate_launch_description():
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')
    pkg = get_package_share_directory('rm2_attachable')

    pkg_ign_plugins_att = get_package_share_directory('attachable_ign_plugin')
    pkg_ign_plugins_contact = get_package_share_directory('contact_ign_plugin')
   
    ign_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
      os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')),
      launch_arguments={'ign_args': ' -r ' + os.path.join(pkg, 'worlds', 'cave_world_robot.sdf')}.items(),)
    
    attachable_pkg = Node(
                     package='rm2_attachable',
                     executable='rosActionServerMerger.py',
                     name='AttachableJointActionServer',
                     output='screen')

    attacher_bridge1 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttacherContact/contact@std_msgs/msg/String@ignition.msgs.StringMsg'],
                      output='screen')

    attacher_bridge2 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttacherContact/touched@std_msgs/msg/Bool@ignition.msgs.Boolean'],
                      output='screen')
                      
    attacher_bridge3 = Node(
                      package='ros_ign_bridge',
                      executable='parameter_bridge',
                      arguments = ['/AttachableJoint@std_msgs/msg/String@ignition.msgs.StringMsg'],
                      output='screen')
    
    cmd_vel_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace='rm2_sim',
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': True
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
                          ],
                          remappings=[
                              ('/rm2_sim/cmd_vel', '/cmd_vel')
                          ])

    
    
    return LaunchDescription([
        SetEnvironmentVariable(name='IGN_GAZEBO_SYSTEM_PLUGIN_PATH',
                                                 value=[(os.path.join(pkg_ign_plugins_att, 'AttachableJoint', 'lib') + ':' 
                                                        + os.path.join(pkg_ign_plugins_contact, 'AttacherContact', 'lib'))]),
        ign_gazebo,
        attachable_pkg,
        attacher_bridge1,
        attacher_bridge2,
        attacher_bridge3,
        cmd_vel_bridge,
    ])

