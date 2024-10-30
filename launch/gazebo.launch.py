import os
  
from ament_index_python.packages import get_package_share_directory
 
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction,IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.event_handlers import OnProcessStart

  
def generate_launch_description():
 
 

    use_sim_time = LaunchConfiguration('use_sim_time') 
    package_name = 'robotpkg'

    
    robot_name_in_model = 'milltap700_5x'
    world = os.path.join(
        get_package_share_directory('robotpkg'),
        'config',
        'empty.world'
        )

    # Get URDF via xacro
    urdf_file_name = 'milltap700_5x.urdf'
    urdf = os.path.join(
        get_package_share_directory('robotpkg'),
        'urdf',
        urdf_file_name
        )
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    robot_description = {"robot_description": robot_desc}
 
    #Actions

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
        )
 
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        name='joint_state_publisher',
    )

    #rivz2
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        parameters=[{'use_sim_time': use_sim_time}],
    )
 
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters= [{'use_sim_time': use_sim_time, 'robot_description': robot_desc}] #[{'use_sim_time': use_sim_time, "robot_description": robot_description_content}],
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', 
        '-s', 'libgazebo_ros_init.so', world], output='screen',
        )


    #spawn the robot 
    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=["-topic", "robot_description", 
                    "-entity", robot_name_in_model,
                    "-x", '0.0',
                    "-y", '0.0',
                    "-z", '0.05',
                    "-Y", '0.0']
    )


    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    controller_params_file = os.path.join(get_package_share_directory(package_name),'config','milltap700_5x.yaml')


    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description,
                    controller_params_file]
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    milltap_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["milltap_controller"],
    )

    delayed_milltap_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[milltap_spawner],
        )
    )

    joint2_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joints2_controllers"],
    )

    joint3_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joints3_controllers"],
    )

    joint4_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joints4_controllers"],
    )

    joint5_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joints5_controllers"],
    )

    joint6_control = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joints6_controllers"],
    )

    delayed_joints_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint2_control, joint3_control, joint4_control, joint5_control, joint6_control],
        )
    )


    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broad_spawner],
        )
    )

     
    return LaunchDescription([
      declare_use_sim_time_cmd,
      start_joint_state_publisher_cmd, 
      robot_state_publisher_node,
      gazebo,
      spawn,
      delayed_controller_manager,
      delayed_joints_spawner,
      delayed_joint_broad_spawner
  ])
