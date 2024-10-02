
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="galil_description",
            description="Description package with robot URDF/xacro files. Usually the argument \
            is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="galil.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Start RViz2 and Joint State Publisher gui automatically \
            with this launch file.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "name",
            default_value='"galil"',
            description="Name of the Galil system",
        )
    )

    # Initialize Arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    gui = LaunchConfiguration("gui")
    name = LaunchConfiguration("name")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " ",
            "name:=",
            name,
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("galil_driver"),
            "config",
            "galil.yaml",
        ]
    )

    # Get RViz config
    rviz_config = PathJoinSubstitution(
        [
            FindPackageShare(description_package),
            "rviz",
            "urdf.rviz",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="both",
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    # joint_state_publisher_node = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     condition=UnlessCondition(LaunchConfiguration('gui'))
    # )
    # joint_state_publisher_node_gui = Node(
    #     package='joint_state_publisher_gui',
    #     executable='joint_state_publisher_gui',
    #     condition=IfCondition(LaunchConfiguration('gui'))
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,

        # joint_state_publisher_node,
        # joint_state_publisher_node_gui,
        rviz_node,
    
    ]

    return LaunchDescription(declared_arguments + nodes)
