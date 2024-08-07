import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, Command, PythonExpression ,TextSubstitution,PathJoinSubstitution
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    robot_name_arg = LaunchConfiguration('robot_name')
    hang_up_arg = LaunchConfiguration('hang_up')
    world_name_arg = LaunchConfiguration('world_name')

    # 条件判断
    # condition_robot_not_unitree_a1 = IfCondition([robot_name_arg, '!=','unitree_a1'])
    # condition_robot_unitree_a1 = IfCondition([robot_name_arg, '==','unitree_a1'])
    #Robot Default Postion
    Cen_x_arg = LaunchConfiguration('Cen_x',default='0.0')
    Cen_y_arg = LaunchConfiguration('Cen_y',default='0.0')
    Cen_z_arg = LaunchConfiguration('Cen_z',default=PythonExpression(["'0.3175' if ", hang_up_arg, " == '0' else '0.5785'"]))
    # Gazebo参数
    paused = LaunchConfiguration('paused', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    debug = LaunchConfiguration('debug', default='false')
    verbose = LaunchConfiguration('verbose', default='true')
    wname = LaunchConfiguration('wname', default='earth')
# <!-- Debug mode will hung up the robot, use "true" or "false" to switch it. -->
    user_debug = LaunchConfiguration('user_debug', default='false')

    # Set the path to the robot_description file
    # 使用 FindPackageShare 和 LaunchConfiguration 来构建路径
    robot_urdf_file_1 = PathJoinSubstitution([FindPackageShare('robot'), 'urdf', robot_name_arg, 'xacro','robot.xacro'])

    # 使用这个路径作为 Command 的一部分
    robot_urdf_command_1 = Command(['xacro ', robot_urdf_file_1])
    # robot_urdf_file_1 = Command(['xacro ', os.path.join(get_package_share_directory('robot'), 'urdf',  TextSubstitution(text=robot_name_arg) , 'xacro', 'robot.xacro')])
    # Load the URDF into the ROS Parameter Server
    spawn_model_node_1 = Node(
        package='gazebo_ros', executable='spawn_model',
        name='spawn_{}_model'.format(robot_name_arg),
        parameters=[{'robot_description': robot_urdf_command_1}],
        arguments=['-urdf', '-param', 'robot_description', '-model', robot_name_arg, '-z', Cen_x_arg, '-x', Cen_y_arg, '-y', Cen_z_arg],
        # condition=condition_robot_not_unitree_a1
    )
    if hang_up_arg == '0' and world_name_arg == 'default':
        box1 = 'box'
    elif hang_up_arg == '1' and world_name_arg == 'default':
        box1 = 'high_box'
    else:
        box1 = world_name_arg
    gazebo_launch_1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/empty_world.launch.py']),
        launch_arguments={
            'world_name': Command(['"', get_package_share_directory('robot'), '/worlds/', box1, '.world"']),
            'paused': paused,
            'use_sim_time': use_sim_time,
            'gui': gui,
            'headless': headless,
            'debug': debug,
            'verbose': verbose
        }.items(),
        # condition=condition_robot_not_unitree_a1
    )
    group_not_unitree_a1 = GroupAction(
        actions=[gazebo_launch_1,spawn_model_node_1],
        # condition=condition_robot_not_unitree_a1
    )


    # gazebo_launch_2 = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([get_package_share_directory('gazebo_ros'), '/launch/empty_world.launch.py']),
    #     launch_arguments={
    #         'world_name': Command(['"', get_package_share_directory('unitree_gazebo'), '/worlds/', wname, '.world"']),
    #         'debug': debug,
    #         'gui': gui,
    #         'paused': paused,
    #         'use_sim_time': use_sim_time,
    #         'headless': headless
    #     }.items(),
    #     condition=condition_robot_unitree_a1
    # )
    # # 设置URDF
    # robot_urdf_file_2 = Command(['xacro ', os.path.join(get_package_share_directory('a1_description'), 'xacro', 'robot.xacro'), ' DEBUG:=',  TextSubstitution(text=user_debug)])

    # 启动URDF模型
    # spawn_model_node_2 = Node(
    #     package='gazebo_ros', executable='spawn_model',
    #     name='urdf_spawner', respawn=False, output='screen',
    #     parameters=[{'robot_description': robot_urdf_file_2}],
    #     arguments=['-urdf', '-z', '0.6', '-model', robot_name_arg, '-param', 'robot_description', '-unpause'],
    #     condition=condition_robot_unitree_a1
    # )
    # group_robot_unitree_a1 = GroupAction(
    #     actions=[gazebo_launch_2,spawn_model_node_2],
    #     condition=condition_robot_unitree_a1
    # )

    ld = LaunchDescription([
        
        DeclareLaunchArgument('robot_name', default_value='elspider3'),  # Set default robot name  elspider3 unitree_a1 elspider_mini
        DeclareLaunchArgument('hang_up', default_value='0'),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('Cen_z', default_value='0.0'),  # Set default z position
        DeclareLaunchArgument('Cen_x', default_value='0.0'),  # Set default x position
        DeclareLaunchArgument('Cen_z', default_value='0.3175', condition=IfCondition(LaunchConfiguration('hang_up') == '0')),
        DeclareLaunchArgument('Cen_z', default_value='0.5785', condition=IfCondition(LaunchConfiguration('hang_up') == '1')),
        DeclareLaunchArgument('world_name', default_value='default'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('user_debug', default_value='false'),
        DeclareLaunchArgument('wname', default_value='earth'),
        group_not_unitree_a1,
        # group_robot_unitree_a1
    ])

    return ld
