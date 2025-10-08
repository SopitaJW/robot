from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    namespace    = LaunchConfiguration('namespace', default='robot')
    prefix       = LaunchConfiguration('prefix', default='')
    gazebo_ignition = LaunchConfiguration('gazebo_ignition', default='true')

    # path to xacro file
    xacro_file = PathJoinSubstitution([
        FindPackageShare('control_rbkairos'),
        'urdf',
        'rbkairos.urdf.xacro'  # หรือชื่อไฟล์จริง เช่น 'rbrobout.urdf.xacro'
    ])

    # Gazebo (Ignition / Garden / Harmonic)
    gz_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r'],
        output='screen'
    )

    # Robot state publisher: generate robot_description from xacro with args
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command([
                FindExecutable(name='xacro'), ' ',
                xacro_file, ' ',
                'namespace:=', namespace, ' ',
                'prefix:=', prefix, ' ',
                'gazebo_ignition:=', gazebo_ignition
            ]),
        }],
        output='screen'
    )

    # spawn robot into Gazebo sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rbkairos'
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('namespace', default_value='robot'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('gazebo_ignition', default_value='true'),
        gz_sim,
        robot_state_publisher,
        spawn_entity,
    ])
