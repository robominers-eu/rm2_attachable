import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace_A = 'rm2_module_A'
    namespace_B = 'rm2_module_B'
    use_sim_time = LaunchConfiguration('use_sim_time')

    # clock bridge
    clock_bridge = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace_A,
                        name='clock_bridge',
                        output='screen',
                        arguments=['/clock' + '@rosgraph_msgs/msg/Clock' + '[ignition.msgs.Clock'],
                        )

    # cmd_vel bridge
    cmd_vel_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace=namespace_A,
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
                          ],
                          remappings=[
                              ('/rm2_module_A/cmd_vel', '/cmd_vel')
                          ])

    cmd_vel_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                          namespace=namespace_B,
                          name='cmd_vel_bridge',
                          output='screen',
                          parameters=[{
                              'use_sim_time': use_sim_time
                          }],
                          arguments=[
                              '/cmd_vel' + '@geometry_msgs/msg/Twist' + ']ignition.msgs.Twist'
                          ],
                          remappings=[
                              ('rm2_module_B/cmd_vel', '/cmd_vel')
                          ])

    # odometry bridge
    odometry_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                           namespace=namespace_A,
                           name='odometry_bridge',
                           output='screen',
                           parameters=[{
                               'use_sim_time': use_sim_time
                           }],
                           arguments=[
                               '/model/rm2_module_A/odometry' +
                               '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
                           ],
                           remappings=[
                               ('/model/rm2_module_A/odometry', '/odom')
                           ])
    
    odometry_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                           namespace=namespace_B,
                           name='odometry_bridge',
                           output='screen',
                           parameters=[{
                               'use_sim_time': use_sim_time
                           }],
                           arguments=[
                               '/model/rm2_module_B/odometry' +
                               '@nav_msgs/msg/Odometry' + '[ignition.msgs.Odometry'
                           ],
                           remappings=[
                               ('/model/rm2_module_B/odometry', '/odom')
                           ])

    # joint state bridge
    joint_state_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                              namespace=namespace_A,
                              name='joint_state_bridge',
                              output='screen',
                              parameters=[{
                                  'use_sim_time': use_sim_time
                              }],
                              arguments=[
                                  '/world/cave_world/model/rm2_module_A/joint_state'
                                  + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model'
                              ],
                              remappings=[
                                  ('/world/cave_world/model/rm2_module_A/joint_state', '/joint_states')
                              ])
    
    joint_state_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                              namespace=namespace_B,
                              name='joint_state_bridge',
                              output='screen',
                              parameters=[{
                                  'use_sim_time': use_sim_time
                              }],
                              arguments=[
                                  '/world/cave_world/model/rm2_module_B/joint_state'
                                  + '@sensor_msgs/msg/JointState' + '[ignition.msgs.Model'
                              ],
                              remappings=[
                                  ('/world/cave_world/model/rm2_module_B/joint_state', '/joint_states')
                              ])

    # lidar bridge
    lidar_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace_A,
                        name='lidar_bridge',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time
                        }],
                        arguments=[
                            '/model/rm2_module_A/laserscan' +
                            '@sensor_msgs/msg/LaserScan' + '[ignition.msgs.LaserScan',
                            '/model/rm2_module_A/laserscanlaserscan/points' +
                            '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
                        ],
                        remappings=[
                            ('/model/rm2_module_A/laserscan/points', '/scan/points'),
                            ('/model/rm2_module_A/laserscan', '/scan')
                        ])

    lidar_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                        namespace=namespace_B,
                        name='lidar_bridge',
                        output='screen',
                        parameters=[{
                            'use_sim_time': use_sim_time
                        }],
                        arguments=[
                            '/model/rm2_module_B/laserscan' +
                            '@sensor_msgs/msg/LaserScan' + '[ignition.msgs.LaserScan',
                            '/model/rm2_module_B/laserscanlaserscan/points' +
                            '@sensor_msgs/msg/PointCloud2' + '[ignition.msgs.PointCloudPacked'
                        ],
                        remappings=[
                            ('/model/rm2_module_B/laserscan/points', '/scan/points'),
                            ('/model/rm2_module_B/laserscan', '/scan')
                        ])
    # color camera bridge
    color_camera_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_A,
                               name='color_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_A/rgbd_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
                               ],
                               remappings=[
                                   ('/model/rm2_module_A/rgbd_camera', '/rgbd_camera')
                               ])
    
    color_camera_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_B,
                               name='color_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_B/rgbd_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image'
                               ],
                               remappings=[
                                   ('/model/rm2_module_B/rgbd_camera', '/rgbd_camera')
                               ])

    pcl2laser_cmd = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laser',
        remappings=[('cloud_in', '/depth_camera/points'),
                    ('scan', '/scan')],
        parameters=[{
            'target_frame': 'rm2_sim/d_435_camera/d435_depth',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.5708,  # -M_PI/2
            'angle_max': 1.5708,  # M_PI/2
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 4.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
    )

    # depth camera bridge
    depth_camera_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_A,
                               name='depth_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_A/depth_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
                                   '/model/rm2_module_A/depth_camera/points' +
                                   '@sensor_msgs/msg/PointCloud2' +
                                   '[ignition.msgs.PointCloudPacked',
                                   '/model/rm2_module_A/camera_info' +
                                   '@sensor_msgs/msg/CameraInfo' + '[ignition.msgs.CameraInfo',
                               ],
                               remappings=[
                                   ('/model/rm2_module_A/depth_camera', '/depth_camera'),
                                   ('/model/rm2_module_A/depth_camera/points', '/depth_camera/points'),
                                   ('/model/rm2_module_A/camera_info', '/camera_info'),
                               ])
    depth_camera_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_B,
                               name='depth_camera_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_B/depth_camera' +
                                   '@sensor_msgs/msg/Image' + '[ignition.msgs.Image',
                                   '/model/rm2_module_B/depth_camera/points' +
                                   '@sensor_msgs/msg/PointCloud2' +
                                   '[ignition.msgs.PointCloudPacked',
                                   '/model/rm2_module_B/camera_info' +
                                   '@sensor_msgs/msg/CameraInfo' + '[ignition.msgs.CameraInfo',
                               ],
                               remappings=[
                                   ('/model/rm2_module_B/depth_camera', '/depth_camera'),
                                   ('/model/rm2_module_B/depth_camera/points', '/depth_camera/points'),
                                   ('/model/rm2_module_B/camera_info', '/camera_info'),
                               ])
    # odom to base_link transform bridge
    odom_base_tf_A = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_A,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_A/tf' +
                                   '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
                               ],
                               remappings=[
                                   ('/model/rm2_module_A/tf', '/tf')
                               ])
    
    odom_base_tf_B = Node(package='ros_ign_bridge', executable='parameter_bridge',
                               namespace=namespace_A,
                               name='odom_base_tf_bridge',
                               output='screen',
                               parameters=[{
                                   'use_sim_time': use_sim_time
                               }],
                               arguments=[
                                   '/model/rm2_module_B/tf' +
                                   '@tf2_msgs/msg/TFMessage' + '[ignition.msgs.Pose_V'
                               ],
                               remappings=[
                                   ('/model/rm2_module_B/tf', '/tf')
                               ])
    


    lidar_stf = Node(package='tf2_ros', executable='static_transform_publisher',
                     namespace=namespace_A,
                     name='lidar_stf',
                     arguments=[
                         '0', '0', '0', '0', '0', '0', '1',
                         'lidar',
                         'rm2_sim/lidar/front_lidar'
                     ])

    camera_stf = Node(package='tf2_ros', executable='static_transform_publisher',
                      namespace=namespace_A,
                      name='camera_stf',
                      arguments=[
                          '0', '0', '0', '0', '0', '0', '1',
                              'd_435_camera',
                              'rm2_sim/d_435_camera/d435_depth'
                      ])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value=['false'],
                              description='use sim time from /clock'),
        clock_bridge,


        cmd_vel_A,  
        odometry_A,
        joint_state_A,
        lidar_A,
        color_camera_A,
        depth_camera_A,
        odom_base_tf_A,

        # cmd_vel_B,
        # odometry_B,
        # joint_state_B,
        # lidar_B,
        # color_camera_B,
        # depth_camera_B,
        # odom_base_tf_B,

 
        pcl2laser_cmd,
        lidar_stf,
        camera_stf,
    ])
