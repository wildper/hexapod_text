import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration,PythonExpression

def generate_launch_description():


    # Get the path to the xacro file
    xacro_file = os.path.join(
        get_package_share_directory('robot'),
        'urdf',
        'elspider_mini',
        'xacro',
        'robot.xacro')
    print('xacro_file',xacro_file)
    Cen_x_arg = LaunchConfiguration('Cen_x',default='0.0')
    Cen_y_arg = LaunchConfiguration('Cen_y',default='0.0')
    Cen_z_arg = LaunchConfiguration('Cen_z',default=PythonExpression('0.5785'))
    # if ", hang_up_arg, " == '0' else '0.5785'"
    # Use xacro to convert to URDF
    # Gazebo参数
    paused = LaunchConfiguration('paused', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false') 
    debug = LaunchConfiguration('debug', default='false')
    verbose = LaunchConfiguration('verbose', default='true')
    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        DeclareLaunchArgument('Cen_x', default_value='0.0'),  # Set default z position
        DeclareLaunchArgument('Cen_y', default_value='0.0'),  # Set default x position
        DeclareLaunchArgument('Cen_z', default_value='0.5785'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('debug', default_value='false'),
        DeclareLaunchArgument('verbose', default_value='true'),
        DeclareLaunchArgument('user_debug', default_value='false'),
        DeclareLaunchArgument('wname', default_value='earth'),
        # Declare the world argument
        DeclareLaunchArgument(
            'world',
            default_value=[os.path.join(get_package_share_directory('robot'), 'world', 'box.world'), ''],
            description='Full path to the world file to load'),

        # Include the Gazebo launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
            launch_arguments={
            'world': LaunchConfiguration('world'),
            'paused': paused,
            'use_sim_time': use_sim_time,
            'gui': gui,
            'headless': headless,
            'debug': debug,
            'verbose': verbose}.items(),
        ),

        # Publish the robot description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description}],
        ),

        # Spawn the robot in Gazebo
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', 'robot_description', '-entity', 'my_robot','-x', Cen_x_arg, '-y', Cen_y_arg, '-z', Cen_z_arg],
            output='screen'),
    ])
